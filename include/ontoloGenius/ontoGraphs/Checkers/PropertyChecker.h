#ifndef PROPERTYCHECKER_H
#define PROPERTYCHECKER_H

#include "ontoloGenius/ontoGraphs/Graphs/PropertyGraph.h"
#include "ontoloGenius/ontoGraphs/Checkers/ValidityChecker.h"

class PropertyChecker : public ValidityChecker<PropertyClassBranch_t>
{
public:
  PropertyChecker(PropertyGraph* graph) : ValidityChecker(graph) {property_graph_ = graph;}
  ~PropertyChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<PropertyClassBranch_t>::printStatus(std::string("property"), std::string("properties"), graph_.size());}
private:
  void checkDisjoint();
  void checkCharacteristics();

  PropertyGraph* property_graph_;
};

#endif
