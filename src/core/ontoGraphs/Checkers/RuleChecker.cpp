#include "ontologenius/core/ontoGraphs/Checkers/RuleChecker.h"

#include <cstddef>
#include <set>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

namespace ontologenius {

  size_t RuleChecker::check()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(rule_graph_->mutex_);

    checkDisjoint();

    is_analysed = true;
    printStatus();

    return getErrors();
  }

  void RuleChecker::checkDisjoint()
  {
    for(RuleBranch* rule_branch : graph_vect_)
    {
      // std::cout << " =========== Analyzing new rule =============" << std::endl;

      current_rule_ = rule_branch->value();
      std::unordered_map<std::string, std::vector<std::vector<ClassElement>>> mapping_var_classes;
      // usage [c1, [c2, [obj_prop1, objprop2]]] ... then check for objprop_n that they are not disjoint
      std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>> mapping_var_obj;
      std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>> mapping_var_data;

      std::set<std::string> keys_variables;
      // check the instantiated atoms in the antecedent and store the variables with atoms
      checkAtomList(&rule_branch->rule_antecedents_, mapping_var_classes, mapping_var_obj, mapping_var_data, keys_variables);
      // check the instantiated atoms in the consequents and store the variables with atoms
      checkAtomList(&rule_branch->rule_consequents_, mapping_var_classes, mapping_var_obj, mapping_var_data, keys_variables);

      // ========== check the mapping between variables in the rule ==========
      checkVariableMappings(mapping_var_classes, keys_variables);
      checkObjectPropertyDisjointess(mapping_var_obj, keys_variables);
      checkDataPropertyDisjointess(mapping_var_data, keys_variables);
    }
  }

  void RuleChecker::checkVariableMappings(std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables)
  {
    for(const auto& var : keys_variables) // loop over each variable in the rules
    {
      auto& var_classes = mapping_var_classes[var];
      for(size_t i = 0; i < var_classes.size(); i++)
      {
        for(size_t j = i + 1; j < var_classes.size(); j++)
        {
          std::vector<std::string> errs;
          errs = checkClassesVectorDisjointness(var_classes[i], var_classes[j]);
          if(errs.empty() == false)
          {
            const std::string err_base = "In rule " + current_rule_ + ": error over variable " + var + " because of ";
            for(const auto& err : errs)
              printError(err_base + err);
          }
        }
      }
    }
  }

  void RuleChecker::checkAtomList(RuleAtomList_t* atom_list, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                                  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>>& mapping_var_obj,
                                  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>>& mapping_var_data,
                                  std::set<std::string>& keys_variables)
  {
    for(auto* atom : atom_list->class_atoms_)
      resolveClassAtom(atom, mapping_var_classes, keys_variables);

    for(auto* atom : atom_list->object_atoms_)
      resolveObjectAtom(atom, mapping_var_classes, mapping_var_obj, keys_variables);

    for(auto* atom : atom_list->data_atoms_)
      resolveDataAtom(atom, mapping_var_classes, mapping_var_data, keys_variables);
  }

  void RuleChecker::resolveClassAtom(ClassAtom_t* atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables)
  {
    std::vector<std::string> errs;

    if(!atom->var.empty())
    {
      keys_variables.insert(atom->var);

      if(atom->class_expression->logical_type_ != logical_none || atom->class_expression->oneof == true || atom->class_expression->is_complex == true) // class expression
      {
        std::vector<ClassElement> expression_domains;
        getUpperLevelDomains(atom->class_expression, expression_domains);

        mapping_var_classes[atom->var].push_back(expression_domains);
      }
      else if(atom->class_expression->object_property_involved_ != nullptr) // single object restriction
      {
        mapping_var_classes[atom->var].push_back(atom->class_expression->object_property_involved_->domains_);
      }
      else if(atom->class_expression->data_property_involved_ != nullptr) // single data restriction
      {
        mapping_var_classes[atom->var].push_back(atom->class_expression->data_property_involved_->domains_);
      }
      else // class only restriction
      {
        std::vector<ClassElement> class_elem;
        class_elem.emplace_back(atom->class_expression->class_involved_);
        mapping_var_classes[atom->var].push_back(class_elem);
      }
    }
    else
    {
      errs = resolveInstantiatedClass(atom->class_expression->class_involved_, atom->individual_involved);
      if(errs.empty() == false)
      {
        const std::string err_base = "In rule " + current_rule_ + ": error over instantiated class atom " +
                                     atom->class_expression->class_involved_->value() + "(" + atom->individual_involved->value() + ")" + " because of ";
        for(const auto& err : errs)
          printError(err_base + err);
      }
    }
  }

  void RuleChecker::resolveObjectAtom(ObjectPropertyAtom_t* atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                                      std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>>& mapping_var_obj, std::set<std::string>& keys_variables)
  {
    std::vector<std::string> errs;

    if(atom->var1.empty() == false)
    {
      keys_variables.insert(atom->var1);
      mapping_var_classes[atom->var1].push_back(atom->object_property_expression->domains_);
    }
    else if(atom->individual_involved_1 != nullptr)
    {
      if(errs.empty() == false)
      {
        errs = resolveInstantiatedObjectProperty(atom->object_property_expression, atom->individual_involved_1, nullptr);
        const std::string err_base = "In rule " + current_rule_ + ": error over instantiated object property atom " + atom->toString() + " on subject " + atom->individual_involved_1->value() + " because of ";
        for(const auto& err : errs)
          printError(err_base + err);
      }
    }

    if(atom->var2.empty() == false)
    {
      keys_variables.insert(atom->var2);
      mapping_var_classes[atom->var2].push_back(atom->object_property_expression->ranges_);
    }
    else if(atom->individual_involved_2 != nullptr)
    {
      if(errs.empty() == false)
      {
        errs = resolveInstantiatedObjectProperty(atom->object_property_expression, nullptr, atom->individual_involved_2);
        const std::string err_base = "In rule " + current_rule_ + ": error over instantiated object property atom " + atom->toString() + " on object " + atom->individual_involved_1->value() + " because of ";
        for(const auto& err : errs)
          printError(err_base + err);
      }
    }

    if(atom->var1.empty() == false && atom->var2.empty() == false)
    {
      mapping_var_obj[atom->var1][atom->var2].push_back(atom->object_property_expression);
    }
  }

  void RuleChecker::resolveDataAtom(DataPropertyAtom_t* atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                                    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>>& mapping_var_data, std::set<std::string>& keys_variables)
  {
    std::vector<std::string> errs;

    if(atom->var1.empty() == false)
    {
      keys_variables.insert(atom->var1);
      mapping_var_classes[atom->var1].push_back(atom->data_property_expression->domains_);
    }
    else if(atom->individual_involved != nullptr)
    {
      if(errs.empty() == false)
      {
        errs = resolveInstantiatedDataProperty(atom->data_property_expression, atom->individual_involved);
        const std::string err_base = "In rule " + current_rule_ + ": error over instantiated data property atom " + atom->toString() +
                                     " on subject " + atom->individual_involved->value() + " because of ";
        for(const auto& err : errs)
          printError(err_base + err);
      }
    }

    if(atom->datatype_involved != nullptr)
    {
      std::string err;
      const std::string err_base = "In rule " + current_rule_ + ": error over instantiated data property atom " + atom->toString() +
                                   " on object " + atom->datatype_involved->value() + " because of ";
      err = checkDataRange(atom->datatype_involved);
      if(err.empty() == false)
        printError(err_base + err);
    }
    else if(atom->var2.empty() == false)
    {
      keys_variables.insert(atom->var2);
    }

    if(atom->var1.empty() == false && atom->var2.empty() == false)
    {
      mapping_var_data[atom->var1][atom->var2].push_back(atom->data_property_expression);
    }
  }

  std::vector<std::string> RuleChecker::resolveInstantiatedClass(ClassBranch* class_branch, IndividualBranch* indiv)
  {
    std::vector<std::string> errs;
    std::string err;

    // check for the same as
    if(indiv->same_as_.empty() == false)
    {
      for(auto& same_elem : indiv->same_as_)
      {
        for(auto& is_a_elem : same_elem.elem->is_a_)
        {
          err = checkBranchDisjointness(class_branch, is_a_elem.elem, rule_graph_->class_graph_);
          if(err.empty() == false)
            errs.push_back(err);
        }
      }
    }
    else
    {
      // check for the indiv
      for(auto& is_a_elem : indiv->is_a_)
      {
        err = checkBranchDisjointness(class_branch, is_a_elem.elem, rule_graph_->class_graph_);
        if(err.empty() == false)
          errs.push_back(err);
      }
    }

    return errs;
  }

  std::vector<std::string> RuleChecker::resolveInstantiatedObjectProperty(ObjectPropertyBranch* property_branch, IndividualBranch* indiv_from, IndividualBranch* indiv_on)
  {
    std::vector<std::string> err;

    if(indiv_from != nullptr)
    {
      err = checkClassesVectorDisjointness(indiv_from->is_a_.relations, property_branch->domains_);
    }
    else if(indiv_on != nullptr)
    {
      err = checkClassesVectorDisjointness(indiv_on->is_a_.relations, property_branch->ranges_);
    }
    return err;
  }

  std::vector<std::string> RuleChecker::resolveInstantiatedDataProperty(DataPropertyBranch* property_branch, IndividualBranch* indiv_from)
  {
    std::vector<std::string> err;

    if(indiv_from != nullptr)
    {
      err = checkClassesVectorDisjointness(indiv_from->is_a_.relations, property_branch->domains_);
    }
    return err;
  }

  void RuleChecker::getUpperLevelDomains(AnonymousClassElement* class_expression, std::vector<ClassElement>& expression_domains)
  {
    // get to the first level which is not a logical node to get the domains of properties or the classes involved.
    // ((hasCamera some Camera) needs to return the domain of hasCamera)

    if(class_expression->logical_type_ == logical_and || class_expression->logical_type_ == logical_or)
    {
      for(auto* sub_elem : class_expression->sub_elements_)
        getUpperLevelDomains(sub_elem, expression_domains);
    }
    else if(class_expression->object_property_involved_ != nullptr)
      for(auto& obj_dom : class_expression->object_property_involved_->domains_)
        expression_domains.push_back(obj_dom);
    else if(class_expression->data_property_involved_ != nullptr)
      for(auto& data_dom : class_expression->data_property_involved_->domains_)
        expression_domains.push_back(data_dom);
    else if(class_expression->class_involved_ != nullptr)
      expression_domains.emplace_back(class_expression->class_involved_);
    else
      return;
  }

  std::vector<std::string> RuleChecker::checkClassesVectorDisjointness(const std::vector<ClassElement>& classes_left, const std::vector<ClassElement>& class_right)
  {
    std::vector<std::string> errs;
    for(const auto& elem_right : class_right)
    {
      for(const auto& elem_left : classes_left)
      {

        const std::string err = checkBranchDisjointness(elem_left.elem, elem_right.elem, rule_graph_->class_graph_);
        if(err.empty() == false)
          errs.push_back(err);
      }
    }
    return errs;
  }

  void RuleChecker::checkObjectPropertyDisjointess(std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>>& mapping_var_obj, std::set<std::string>& keys_variables)
  {
    std::string err;
    const std::string err_base = "In rule " + current_rule_ + ": error over object property atom" + " because of ";
    // map is [var1][var2][obj_prop1, ..., objpropn]
    //               ...
    //              [varn][obj_prop1, ..., objpropn]
    //        [var2][var1][obj_prop1, ..., objpropn]

    for(auto& var_elem1 : keys_variables) // loop over variables in map
    {
      for(auto& var_elem2 : keys_variables) // loop 2nd time over variables in map
      {
        auto& obj_elems = mapping_var_obj[var_elem1][var_elem2];
        for(size_t i = 0; i < obj_elems.size(); i++)
        {
          for(size_t j = i + 1; j < obj_elems.size(); j++)
          {
            err = checkBranchDisjointness(obj_elems[i], obj_elems[j], rule_graph_->object_property_graph_);
            if(err.empty() == false)
              printError(err_base + err);
          }
        }
      }
    }
  }

  void RuleChecker::checkDataPropertyDisjointess(std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>>& mapping_var_data, std::set<std::string>& keys_variables)
  {
    std::string err;
    const std::string err_base = "In rule  " + current_rule_ + ": error over data property atom" + " because of ";
    // map is [var1][var2][obj_prop1, ..., objpropn]
    //               ...
    //              [varn][obj_prop1, ..., objpropn]
    //        [var2][var1][obj_prop1, ..., objpropn]

    for(auto& var_elem1 : keys_variables) // loop over variables in map
    {
      for(auto& var_elem2 : keys_variables) // loop 2nd time over variables in map
      {
        auto& data_elems = mapping_var_data[var_elem1][var_elem2];
        for(size_t i = 0; i < data_elems.size(); i++)
        {
          for(size_t j = i + 1; j < data_elems.size(); j++)
          {
            err = checkBranchDisjointness(data_elems[i], data_elems[j], rule_graph_->data_property_graph_);
            if(err.empty() == false)
              printError(err_base + err);
          }
        }
      }
    }
  }

  std::string RuleChecker::checkDataRange(LiteralNode* datatype_involved) // au niveau de LiteralNode
  {
    std::string error;

    if(datatype_involved->type_ == "boolean")
    {
      if((datatype_involved->value_ != "false") && (datatype_involved->value_ != "true"))
      {
        error = " unmatching data type " + datatype_involved->type_ + " and data value " + datatype_involved->value_;
      }
    }
    else if(datatype_involved->type_ == "integer")
    {
      try
      {
        const int conv_val = std::stoi(datatype_involved->value_);
      }
      catch(std::invalid_argument const& ex)
      {
        error = " unmatching data type " + datatype_involved->type_ + " and data value " + datatype_involved->value_;
      }
    }
    else if(datatype_involved->type_ == "real")
    {
      try
      {
        const float conv_val = std::stof(datatype_involved->value_);
      }
      catch(std::invalid_argument const& ex)
      {
        error = " unmatching data type " + datatype_involved->type_ + " and data value " + datatype_involved->value_;
      }
    }
    else if(datatype_involved->type_ == "double")
    {
      try
      {
        const double conv_val = std::stod(datatype_involved->value_);
      }
      catch(std::invalid_argument const& ex)
      {
        error = " unmatching data type " + datatype_involved->type_ + " and data value " + datatype_involved->value_;
      }
    }

    return error;
  }
} // namespace ontologenius