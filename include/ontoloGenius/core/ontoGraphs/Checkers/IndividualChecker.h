#ifndef INDIVIDUALCHECKER_H
#define INDIVIDUALCHECKER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontoloGenius/core/ontoGraphs/Checkers/ValidityChecker.h"

class IndividualChecker : public ValidityChecker<IndividualBranch_t>
{
public:
  IndividualChecker(IndividualGraph* graph) : ValidityChecker(graph) {individual_graph_ = graph;}
  ~IndividualChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<IndividualBranch_t>::printStatus(std::string("individual"), std::string("individuals"), graph_vect_.size());}
private:
  IndividualGraph* individual_graph_;

  void checkSame();
  void checkReflexive();
  void checkObectPropertyDomain();
  void checkObectPropertyRange();

  void checkDataPropertyDomain();
  void checkDataPropertyRange();

  void checkAssymetric();

  bool symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
};

#endif
