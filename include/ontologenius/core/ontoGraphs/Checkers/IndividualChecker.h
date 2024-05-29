#ifndef ONTOLOGENIUS_INDIVIDUALCHECKER_H
#define ONTOLOGENIUS_INDIVIDUALCHECKER_H

#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

  class IndividualChecker : public ValidityChecker<IndividualBranch>
  {
  public:
    explicit IndividualChecker(IndividualGraph* graph) : ValidityChecker(graph) { individual_graph_ = graph; }
    ~IndividualChecker() = default;

    size_t check() override;
    void printStatus() { ValidityChecker<IndividualBranch>::printStatus("individual", "individuals", graph_vect_.size()); }

  private:
    IndividualGraph* individual_graph_;

    void checkDisjointInheritance(IndividualBranch* indiv, std::unordered_set<ClassBranch*> ups);

    void checkDisjoint(IndividualBranch* indiv);
    void checkReflexive(IndividualBranch* indiv);

    void checkObectRelations(IndividualBranch* indiv, std::unordered_set<ClassBranch*> up_from);
    void checkDataRelations(IndividualBranch* indiv, std::unordered_set<ClassBranch*> up_from);

    void checkAssymetric(IndividualBranch* indiv);

    bool symetricExist(IndividualBranch* indiv_on, ObjectPropertyBranch* sym_prop, IndividualBranch* sym_indiv);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALCHECKER_H
