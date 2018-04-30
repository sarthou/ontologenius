#ifndef DATAPROPERTYCHECKER_H
#define DATAPROPERTYCHECKER_H

#include "ontoloGenius/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontoloGenius/ontoGraphs/Checkers/ValidityChecker.h"

class DataPropertyChecker : public ValidityChecker<DataPropertyBranch_t>
{
public:
  DataPropertyChecker(DataPropertyGraph* graph) : ValidityChecker(graph) {property_graph_ = graph;}
  ~DataPropertyChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<DataPropertyBranch_t>::printStatus(std::string("data property"), std::string("data properties"), graph_.size());}
private:
  void checkDisjoint();

  DataPropertyGraph* property_graph_;
};

#endif
