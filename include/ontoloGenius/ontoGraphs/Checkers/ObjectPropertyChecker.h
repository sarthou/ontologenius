#ifndef OBJECTPROPERTYCHECKER_H
#define OBJECTPROPERTYCHECKER_H

#include "ontoloGenius/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/ontoGraphs/Checkers/ValidityChecker.h"

class ObjectPropertyChecker : public ValidityChecker<ObjectPropertyBranch_t>
{
public:
  ObjectPropertyChecker(ObjectPropertyGraph* graph) : ValidityChecker(graph) {property_graph_ = graph;}
  ~ObjectPropertyChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<ObjectPropertyBranch_t>::printStatus(std::string("object property"), std::string("object properties"), graph_.size());}
private:
  void checkDisjoint();
  void checkCharacteristics();

  ObjectPropertyGraph* property_graph_;
};

#endif
