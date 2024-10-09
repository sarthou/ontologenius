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
