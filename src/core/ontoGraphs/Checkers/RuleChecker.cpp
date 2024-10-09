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
