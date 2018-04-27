#ifndef PROPERTYCHECKER_H
#define PROPERTYCHECKER_H

#include "ontoloGenius/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/ontoGraphs/Checkers/ValidityChecker.h"

class PropertyChecker : public ValidityChecker<ObjectPropertyBranch_t>
{
public:
  PropertyChecker(ObjectPropertyGraph* graph) : ValidityChecker(graph) {property_graph_ = graph;}
  ~PropertyChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<ObjectPropertyBranch_t>::printStatus(std::string("property"), std::string("properties"), graph_.size());}
private:
  void checkDisjoint();
  void checkCharacteristics();

  ObjectPropertyGraph* property_graph_;
};

#endif
