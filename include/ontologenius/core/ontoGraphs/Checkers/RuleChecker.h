#ifndef ONTOLOGENIUS_RULECHECKER_H
#define ONTOLOGENIUS_RULECHECKER_H

#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

namespace ontologenius {

  class RuleChecker : public ValidityChecker<RuleBranch>
  {
  public:
    explicit RuleChecker(RuleGraph* graph) : ValidityChecker(graph), rule_graph_(graph) {}
    ~RuleChecker() override = default;

    size_t check() override;
    void checkDisjoint();

    void printStatus() override { ValidityChecker<RuleBranch>::printStatus("rule", "rules", graph_vect_.size()); }

  private:
    RuleGraph* rule_graph_;
    std::string current_rule_;

    void checkAtomList(RuleAtomList_t* atom_list, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                       std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>>& mapping_var_obj,
                       std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>>& mapping_var_data,
                       std::set<std::string>& keys_variables);
    void checkVariableMappings(std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables);

    std::vector<std::string> resolveInstantiatedClass(ClassBranch* class_branch, IndividualBranch* indiv);
    std::vector<std::string> resolveInstantiatedObjectProperty(ObjectPropertyBranch* property_branch, IndividualBranch* indiv_from, IndividualBranch* indiv_on);
    std::vector<std::string> resolveInstantiatedDataProperty(DataPropertyBranch* property_branch, IndividualBranch* indiv_from);

    void resolveClassAtom(ClassAtom_t* class_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables);
    void resolveObjectAtom(ObjectPropertyAtom_t* object_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                           std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>>& mapping_var_obj, std::set<std::string>& keys_variables);
    void resolveDataAtom(DataPropertyAtom_t* data_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes,
                         std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>>& mapping_var_data, std::set<std::string>& keys_variables);
    // void resolveBuiltinAtom(ClassAtom_t* class_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes);

    void getUpperLevelDomains(AnonymousClassElement* class_expression, std::vector<ClassElement>& expression_domains);

    std::string checkClassesDisjointness(ClassBranch* class_left, ClassBranch* class_right);
    std::vector<std::string> checkClassesVectorDisjointness(const std::vector<ClassElement>& classes_left, const std::vector<ClassElement>& class_right);

    std::string checkObjectPropertyDisjointess(ObjectPropertyBranch* branch_left, ObjectPropertyBranch* branch_right);
    void checkObjectPropertyDisjointess(std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ObjectPropertyBranch*>>>& mapping_var_obj,
                                        std::set<std::string>& keys_variables);

    std::string checkDataPropertyDisjointess(DataPropertyBranch* branch_left, DataPropertyBranch* branch_right);
    void checkDataPropertyDisjointess(std::unordered_map<std::string, std::unordered_map<std::string, std::vector<DataPropertyBranch*>>>& mapping_var_data,
                                      std::set<std::string>& keys_variables);
    std::string checkDataRange(LiteralNode* datatype_involved);

    template<typename T, typename G>
    std::string checkBranchDisjointness(T* branch_left, T* branch_right, G* graph_)
    {
      std::string err;
      std::unordered_set<T*> disjoints;

      graph_->getDisjoint(branch_left, disjoints);

      T* first_crash = nullptr;
      if(disjoints.empty() == false)
      {
        std::unordered_set<T*> ups;
        graph_->getUpPtr(branch_right, ups);
        first_crash = graph_->firstIntersection(ups, disjoints);
      }

      if(first_crash != nullptr)
      {
        std::unordered_set<T*> intersection_ups;
        graph_->getUpPtr(first_crash, intersection_ups);
        std::unordered_set<T*> left_ups;
        graph_->getUpPtr(branch_left, left_ups);

        T* explanation_1 = nullptr;
        T* explanation_2 = nullptr;
        for(auto* up : intersection_ups)
        {
          explanation_2 = up;
          explanation_1 = graph_->firstIntersection(left_ups, up->disjoints_);
          if(explanation_1 != nullptr)
            break;
        }

        std::string exp_str;
        if(branch_right != explanation_2)
          exp_str = branch_right->value() + " is a " + explanation_2->value();
        if(branch_left != explanation_1)
        {
          if(exp_str.empty() == false)
            exp_str += " and ";
          exp_str += branch_left->value() + " is a " + explanation_1->value();
        }

        if(explanation_1 == nullptr)
          err = "disjointness between " + branch_left->value() + " and " + branch_right->value() +
                " over " + first_crash->value();
        else if(exp_str.empty() == false)
          err = "disjointness between " + branch_left->value() + " and " + branch_right->value() +
                " because " + explanation_1->value() + " and " + explanation_2->value() + " are disjoint" +
                " and " + exp_str;
        else
          err = "disjointness between " + branch_left->value() + " and " + branch_right->value() +
                " because " + explanation_1->value() + " and " + explanation_2->value() + " are disjoint";
      }

      return err;
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_RULECHECKER_H