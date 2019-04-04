#ifndef DATAPROPERTYCHECKER_H
#define DATAPROPERTYCHECKER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Checkers/ValidityChecker.h"

namespace ontologenius {

class DataPropertyChecker : public ValidityChecker<DataPropertyBranch_t>
{
public:
  DataPropertyChecker(DataPropertyGraph* graph) : ValidityChecker(graph) {property_graph_ = graph;}
  ~DataPropertyChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<DataPropertyBranch_t>::printStatus(std::string("data property"), std::string("data properties"), graph_vect_.size());}
private:
  void checkDisjoint();

  DataPropertyGraph* property_graph_;
};

} // namespace ontologenius

#endif
