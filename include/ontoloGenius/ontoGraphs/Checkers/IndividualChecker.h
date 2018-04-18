#ifndef INDIVIDUALCHECKER_H
#define INDIVIDUALCHECKER_H

#include "ontoloGenius/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontoloGenius/ontoGraphs/Checkers/ValidityChecker.h"

class IndividualChecker : public ValidityChecker<IndividualBranch_t>
{
public:
  IndividualChecker(IndividualGraph* graph) : ValidityChecker(graph) {individual_graph_ = graph;}
  ~IndividualChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<IndividualBranch_t>::printStatus(std::string("individual"), std::string("individuals"), graph_.size());}
private:
  IndividualGraph* individual_graph_;

  void checkSame();
  void checkReflexive();
  void checkDomain();
  void checkRange();
  void checkAssymetric();

  bool symetricExist(IndividualBranch_t* indiv_on, PropertyClassBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
};

#endif
