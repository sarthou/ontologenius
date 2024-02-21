#ifndef ONTOLOGENIUS_INDIVIDUALCHECKER_H
#define ONTOLOGENIUS_INDIVIDUALCHECKER_H

#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"

namespace ontologenius {

class IndividualChecker : public ValidityChecker<IndividualBranch_t>
{
public:
  explicit IndividualChecker(IndividualGraph* graph) : ValidityChecker(graph) {individual_graph_ = graph;}
  ~IndividualChecker() {}

  size_t check() override;
  void printStatus(){ValidityChecker<IndividualBranch_t>::printStatus("individual", "individuals", graph_vect_.size());}
private:
  IndividualGraph* individual_graph_;

  void checkDisjointInheritance(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*> ups);

  void checkDisjoint(IndividualBranch_t* indiv);
  void checkReflexive(IndividualBranch_t* indiv);

  void checkObectRelations(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*> up_from);
  void checkDataRelations(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*> up_from);

  void checkAssymetric(IndividualBranch_t* indiv);

  bool symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALCHECKER_H
