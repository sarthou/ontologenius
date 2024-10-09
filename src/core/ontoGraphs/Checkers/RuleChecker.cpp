#include "ontologenius/core/ontoGraphs/Checkers/RuleChecker.h"

#include <set>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
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
      std::cout << " =========== Analyzing new rule =============" << std::endl;

      std::unordered_map<std::string, std::vector<std::vector<ClassElement>>> mapping_var_classes;
      std::set<std::string> keys_variables;
      // check the instantiated atoms in the antecedent and store the variables with atoms
      checkAtomList(&rule_branch->rule_antecedents_, mapping_var_classes, keys_variables);
      // check the instantiated atoms in the consequents and store the variables with atoms
      checkAtomList(&rule_branch->rule_consequents_, mapping_var_classes, keys_variables);

      // ========== check the mapping between variables in the rule ==========
      checkVariableMappings(mapping_var_classes, keys_variables);

    }
  }

  void RuleChecker::checkVariableMappings(std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables)
  {
    for(auto& var : keys_variables) // loop over each variable in the rules
    {
      std::cout << "analyzing variable : " << var << std::endl;
      for(auto& domain_elem_1 : mapping_var_classes[var]) // loop over the the vector of ClassElement to test a 1st time
      {
        for(auto& domain_elem_2 : mapping_var_classes[var]) // loop over the the vector of ClassElement to test a 2nd time
        {
          if(domain_elem_1 != domain_elem_2) // if they are different, compare them to check if they are disjoint
          {
            std::vector<std::string> exp;
            exp = checkClassesVectorDisjointness(domain_elem_1, domain_elem_2);
            if(exp.empty() == false)
              std::cout << "disjointess on var :" << var << std::endl;
          }
        }
      }
    }
  }

  void RuleChecker::checkAtomList(RuleAtomList* atom_list, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables)
  {
    for(auto* atom : atom_list->class_atoms_)
      resolveClassAtom(atom, mapping_var_classes, keys_variables);

    for(auto* atom : atom_list->object_atoms_)
      resolveObjectAtom(atom, mapping_var_classes, keys_variables);

    for(auto* atom : atom_list->data_atoms_)
      resolveDataAtom(atom, mapping_var_classes, keys_variables);

    // for(auto* atom : atom_list->builtin_atoms_)
    //   resolveBuiltinAtom(atom, mapping_var_classes);
  }

  void RuleChecker::resolveClassAtom(ClassAtom* atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables)
  {
    std::cout << "resolveClassAtom " << std::endl;
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
        class_elem.push_back(ClassElement{atom->class_expression->class_involved_});
        mapping_var_classes[atom->var].push_back(class_elem);
      }
    }
    else
      errs = resolveInstantiatedClassAtom(atom->class_expression->class_involved_, atom->individual_involved);
  }

  void RuleChecker::resolveObjectAtom(ObjectPropertyAtom* atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables)
  {
    std::vector<std::string> errs;
    std::cout << "resolveObjectAtom " << std::endl;

    if(atom->var1.empty() == false)
    {
      keys_variables.insert(atom->var1);
      mapping_var_classes[atom->var1].push_back(atom->object_property_expression->domains_);
    }
    else if(atom->individual_involved_1 != nullptr)
    {
      errs = resolveInstantiatedObjectPropertyAtom(atom->object_property_expression, atom->individual_involved_1, nullptr);
    }

    if(atom->var2.empty() == false)
    {
      keys_variables.insert(atom->var2);
      mapping_var_classes[atom->var2].push_back(atom->object_property_expression->ranges_);
    }
    else if(atom->individual_involved_2 != nullptr)
    {
      errs = resolveInstantiatedObjectPropertyAtom(atom->object_property_expression, nullptr, atom->individual_involved_2);
    }
  }

  void RuleChecker::resolveDataAtom(DataPropertyAtom* atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables)
  {
    std::vector<std::string> errs;
    std::cout << "resolveDataAtom " << std::endl;

    if(atom->var1.empty() == false)
    {
      keys_variables.insert(atom->var1);
      mapping_var_classes[atom->var1].push_back(atom->data_property_expression->domains_);
    }
    else if(atom->individual_involved != nullptr)
    {
      errs = resolveInstantiatedDataPropertyAtom(atom->data_property_expression, atom->individual_involved);
    }

    // TODO check var2 and Literal var;
  }

  std::vector<std::string> RuleChecker::resolveInstantiatedClassAtom(ClassBranch* class_branch, IndividualBranch* indiv)
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
          err = checkClassesDisjointness(class_branch, is_a_elem.elem);
          if(err.empty() == false)
            errs.push_back(err);
        }
      }
    }

    // check for the indiv
    for(auto& is_a_elem : indiv->is_a_)
    {
      err = checkClassesDisjointness(class_branch, is_a_elem.elem);
      if(err.empty() == false)
        errs.push_back(err);
    }

    return errs;
  }

  std::vector<std::string> RuleChecker::resolveInstantiatedObjectPropertyAtom(ObjectPropertyBranch* property_branch, IndividualBranch* indiv_from, IndividualBranch* indiv_on)
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

  std::vector<std::string> RuleChecker::resolveInstantiatedDataPropertyAtom(DataPropertyBranch* property_branch, IndividualBranch* indiv_from)
  {
    std::vector<std::string> err;
    // std::vector<LiteralNode*> range_prop = data_property->ranges_;

    if(indiv_from != nullptr)
    {
      err = checkClassesVectorDisjointness(indiv_from->is_a_.relations, property_branch->domains_);
      // if(err.empty() == false)
      //   errs.push_back(err);
    }
    // else if(indiv_on != nullptr)
    // {
    //   err = checkClassesVectorDisjointness(indiv_on->is_a_.relations, range_prop);
    //   if(err.empty() == false)
    //     errs.push_back(err);
    // }
    return err;
  }

  void RuleChecker::getUpperLevelDomains(AnonymousClassElement* class_expression, std::vector<ClassElement>& expression_domains)
  {
    // get to the first level which is not a logical node to get the domains of properties or the classes involved.
    // ((hasCamera some Camera) needs to return the domain of hasCamera)

    if(class_expression->logical_type_ == logical_and || class_expression->logical_type_ == logical_or)
    {
      for(auto sub_elem : class_expression->sub_elements_)
        getUpperLevelDomains(sub_elem, expression_domains);
    }
    else if(class_expression->object_property_involved_ != nullptr)
      for(auto& obj_dom : class_expression->object_property_involved_->domains_)
        expression_domains.push_back(obj_dom);
    else if(class_expression->data_property_involved_ != nullptr)
      for(auto& data_dom : class_expression->data_property_involved_->domains_)
        expression_domains.push_back(data_dom);
    else if(class_expression->class_involved_ != nullptr)
      expression_domains.push_back(ClassElement{class_expression->class_involved_});
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
        const std::string err = checkClassesDisjointness(elem_left.elem, elem_right.elem);
        if(err.empty() == false)
          errs.push_back(err);
      }
    }
    return errs;
  }

  std::string RuleChecker::checkClassesDisjointness(ClassBranch* class_left, ClassBranch* class_right)
  {
    std::string err;
    std::unordered_set<ClassBranch*> disjoints;

    rule_graph_->class_graph_->getDisjoint(class_left, disjoints);

    ClassBranch* first_crash = nullptr;
    if(disjoints.empty() == false)
    {
      std::unordered_set<ClassBranch*> ups;
      rule_graph_->class_graph_->getUpPtr(class_right, ups);
      first_crash = rule_graph_->class_graph_->firstIntersection(ups, disjoints);
    }

    if(first_crash != nullptr)
    {
      std::unordered_set<ClassBranch*> intersection_ups;
      rule_graph_->class_graph_->getUpPtr(first_crash, intersection_ups);
      std::unordered_set<ClassBranch*> left_ups;
      rule_graph_->class_graph_->getUpPtr(class_left, left_ups);

      ClassBranch* explanation_1 = nullptr;
      ClassBranch* explanation_2 = nullptr;
      for(auto* up : intersection_ups)
      {
        explanation_2 = up;
        explanation_1 = rule_graph_->class_graph_->firstIntersection(left_ups, up->disjoints_);
        if(explanation_1 != nullptr)
          break;
      }

      std::string exp_str;
      if(class_right != explanation_2)
        exp_str = class_right->value() + " is a " + explanation_2->value();
      if(class_left != explanation_1)
      {
        if(exp_str.empty() == false)
          exp_str += " and ";
        exp_str += class_left->value() + " is a " + explanation_1->value();
      }

      if(explanation_1 == nullptr)
        err = "disjointness between " + class_left->value() + " and " + class_right->value() +
              " over " + first_crash->value();
      else if(exp_str.empty() == false)
        err = "disjointness between " + class_left->value() + " and " + class_right->value() +
              " because " + explanation_1->value() + " and " + explanation_2->value() + " are disjoint" +
              " and " + exp_str;
      else
        err = "disjointness between " + class_left->value() + " and " + class_right->value() +
              " because " + explanation_1->value() + " and " + explanation_2->value() + " are disjoint";
    }

    return err;
  }

} // namespace ontologenius