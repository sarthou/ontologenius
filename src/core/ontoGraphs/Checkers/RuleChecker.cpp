#include "ontologenius/core/ontoGraphs/Checkers/RuleChecker.h"

#include <cstddef>
#include <set>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
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
      current_rule_ = rule_branch->value();
      std::unordered_map<std::string, std::vector<std::vector<ClassElement>>> mapping_var_classes;
      // usage [c1, [c2, [obj_prop1, objprop2]]] ... then check for objprop_n that they are not disjoint
      std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>> mapping_var_obj;
      std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>> mapping_var_data;

      std::set<std::string> keys_variables;

      checkAtomList(rule_branch->rule_body_, mapping_var_classes, mapping_var_obj, mapping_var_data, keys_variables);
      checkAtomList(rule_branch->rule_head_, mapping_var_classes, mapping_var_obj, mapping_var_data, keys_variables);

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

  void RuleChecker::checkAtomList(std::vector<RuleTriplet_t>& atoms_list, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                                  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>>& mapping_var_obj,
                                  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>>& mapping_var_data,
                                  std::set<std::string>& keys_variables)
  {
    for(auto& atom : atoms_list)
    {
      switch(atom.atom_type_)
      {
      case class_atom:
        resolveClassTriplet(atom, mapping_var_classes, keys_variables);
        break;
      case object_atom:
        resolveObjectTriplet(atom, mapping_var_classes, mapping_var_obj, keys_variables);
        break;
      case data_atom:
        resolveDataTriplet(atom, mapping_var_classes, mapping_var_data, keys_variables);
        break;
      case builtin_atom:
        /* code */
        break;

      default:
        break;
      }
    }
  }

  void RuleChecker::resolveClassTriplet(RuleTriplet_t& class_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables)
  {
    std::vector<std::string> errs;

    if(class_atom.subject.is_variable)
    {
      keys_variables.insert(class_atom.subject.name);
      if(class_atom.class_element != nullptr)
      {
        if(class_atom.class_element->logical_type_ != logical_none || class_atom.class_element->oneof == true || class_atom.class_element->is_complex == true) // class expression
        {
          std::vector<ClassElement> expression_domains;
          getUpperLevelDomains(class_atom.class_element, expression_domains);
          mapping_var_classes[class_atom.subject.name].push_back(expression_domains);
        }
        else if(class_atom.class_element->object_property_involved_ != nullptr) // single object restriction
          mapping_var_classes[class_atom.subject.name].push_back(class_atom.class_element->object_property_involved_->domains_);
        else if(class_atom.class_element->data_property_involved_ != nullptr) // single data restriction
          mapping_var_classes[class_atom.subject.name].push_back(class_atom.class_element->data_property_involved_->domains_);
      }
      else // class only restriction
      {
        std::vector<ClassElement> class_elem;
        class_elem.emplace_back(class_atom.class_predicate);
        mapping_var_classes[class_atom.subject.name].push_back(class_elem);
      }
    }
    else
    {
      errs = resolveInstantiatedClass(class_atom.class_predicate, class_atom.subject.indiv_value);
      if(errs.empty() == false)
      {
        const std::string err_base = "In rule " + current_rule_ + ": error over instantiated class atom " +
                                     class_atom.class_predicate->value() + "(" + class_atom.subject.indiv_value->value() + ")" + " because of ";
        for(const auto& err : errs)
          printError(err_base + err);
      }
    }
  }

  void RuleChecker::resolveObjectTriplet(RuleTriplet_t& object_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                                         std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>>& mapping_var_obj, std::set<std::string>& keys_variables)
  {
    std::vector<std::string> errs;
    if(object_atom.subject.is_variable)
    {
      keys_variables.insert(object_atom.subject.name);
      mapping_var_classes[object_atom.subject.name].push_back(object_atom.object_predicate->domains_);
    }
    else if(object_atom.subject.indiv_value != nullptr)
    {
      if(errs.empty() == false)
      {
        errs = resolveInstantiatedObjectProperty(object_atom.object_predicate, object_atom.subject.indiv_value, nullptr);
        const std::string err_base = "In rule " + current_rule_ + ": error over instantiated object property atom " + object_atom.toString() + " on subject " + object_atom.subject.indiv_value->value() + " because of ";
        for(const auto& err : errs)
          printError(err_base + err);
      }
    }

    if(object_atom.object.is_variable)
    {
      keys_variables.insert(object_atom.object.name);
      mapping_var_classes[object_atom.object.name].push_back(object_atom.object_predicate->ranges_);
    }
    else if(object_atom.object.indiv_value != nullptr)
    {
      if(errs.empty() == false)
      {
        errs = resolveInstantiatedObjectProperty(object_atom.object_predicate, nullptr, object_atom.object.indiv_value);
        const std::string err_base = "In rule " + current_rule_ + ": error over instantiated object property atom " + object_atom.toString() + " on object " + object_atom.object.indiv_value->value() + " because of ";
        for(const auto& err : errs)
          printError(err_base + err);
      }
    }

    if(object_atom.subject.is_variable && object_atom.object.is_variable)
      mapping_var_obj[object_atom.subject.name][object_atom.object.name].push_back(object_atom.object_predicate);
  }

  void RuleChecker::resolveDataTriplet(RuleTriplet_t& data_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                                       std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>>& mapping_var_data, std::set<std::string>& keys_variables)
  {
    std::vector<std::string> errs;

    if(data_atom.subject.is_variable)
    {
      keys_variables.insert(data_atom.subject.name);
      mapping_var_classes[data_atom.subject.name].push_back(data_atom.data_predicate->domains_);
    }
    else if(data_atom.subject.indiv_value != nullptr)
    {
      if(errs.empty() == false)
      {
        errs = resolveInstantiatedDataProperty(data_atom.data_predicate, data_atom.subject.indiv_value);
        const std::string err_base = "In rule " + current_rule_ + ": error over instantiated data property atom " + data_atom.toString() +
                                     " on subject " + data_atom.subject.indiv_value->value() + " because of ";
        for(const auto& err : errs)
          printError(err_base + err);
      }
    }

    if(data_atom.object.is_variable)
      keys_variables.insert(data_atom.object.name);
    else if(data_atom.object.datatype_value != nullptr)
    {
      std::string err;
      const std::string err_base = "In rule " + current_rule_ + ": error over instantiated data property atom " + data_atom.toString() +
                                   " on object " + data_atom.object.datatype_value->value() + " because of ";
      err = checkDataRange(data_atom.object.datatype_value);
      if(err.empty() == false)
        printError(err_base + err);
    }

    if(data_atom.subject.is_variable && data_atom.object.is_variable)
      mapping_var_data[data_atom.subject.name][data_atom.object.name].push_back(data_atom.data_predicate);
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
      err = checkClassesVectorDisjointness(indiv_from->is_a_.relations, property_branch->domains_);
    else if(indiv_on != nullptr)
      err = checkClassesVectorDisjointness(indiv_on->is_a_.relations, property_branch->ranges_);
    return err;
  }

  std::vector<std::string> RuleChecker::resolveInstantiatedDataProperty(DataPropertyBranch* property_branch, IndividualBranch* indiv_from)
  {
    std::vector<std::string> err;

    if(indiv_from != nullptr)
      err = checkClassesVectorDisjointness(indiv_from->is_a_.relations, property_branch->domains_);
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

    for(const auto& var_elem1 : keys_variables) // loop over variables in map
    {
      for(const auto& var_elem2 : keys_variables) // loop 2nd time over variables in map
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

    for(const auto& var_elem1 : keys_variables) // loop over variables in map
    {
      for(const auto& var_elem2 : keys_variables) // loop 2nd time over variables in map
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
        error = " unmatching data type " + datatype_involved->type_ + " and data value " + datatype_involved->value_;
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