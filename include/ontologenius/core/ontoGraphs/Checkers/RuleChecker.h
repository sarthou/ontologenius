#ifndef ONTOLOGENIUS_RULECHECKER_H
#define ONTOLOGENIUS_RULECHECKER_H

#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

namespace ontologenius {

  class RuleChecker : public ValidityChecker<RuleBranch>
  {
  public:
    explicit RuleChecker(RuleGraph* graph) : ValidityChecker(graph) { rule_graph_ = graph; }
    ~RuleChecker() {}

    size_t check() override;
    void checkDisjoint();

    void printStatus()
    {
      ValidityChecker<RuleBranch>::printStatus("rule", "rules", graph_vect_.size());
    };

  private:
    RuleGraph* rule_graph_;
    std::string current_rule_;

    void checkAtomList(RuleAtomList_t* atom_list, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables);
    void checkVariableMappings(std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables);

    std::vector<std::string> resolveInstantiatedClass(ClassBranch* class_branch, IndividualBranch* indiv);
    std::vector<std::string> resolveInstantiatedObjectProperty(ObjectPropertyBranch* property_branch, IndividualBranch* indiv_from, IndividualBranch* indiv_on);
    std::vector<std::string> resolveInstantiatedDataProperty(DataPropertyBranch* data_property, IndividualBranch* indiv_from);

    void resolveClassAtom(ClassAtom_t* class_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables);
    void resolveObjectAtom(ObjectPropertyAtom_t* object_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables);
    void resolveDataAtom(DataPropertyAtom_t* data_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes, std::set<std::string>& keys_variables);
    // void resolveBuiltinAtom(ClassAtom_t* class_atom, std::unordered_map<std::string, std::vector<std::vector<ClassElement>>>& mapping_var_classes);

    void getUpperLevelDomains(AnonymousClassElement* class_expression, std::vector<ClassElement>& expression_domains);
    std::string checkClassesDisjointness(ClassBranch* class_left, ClassBranch* class_right);
    std::vector<std::string> checkClassesVectorDisjointness(const std::vector<ClassElement>& classes_left, const std::vector<ClassElement>& class_right);
    std::string checkDataRange(LiteralNode* data_node);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_RULECHECKER_H