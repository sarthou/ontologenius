#ifndef ONTOLOGENIUS_OBJECTPROPERTYCHECKER_H
#define ONTOLOGENIUS_OBJECTPROPERTYCHECKER_H

#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"

namespace ontologenius {

class ObjectPropertyChecker : public ValidityChecker<ObjectPropertyBranch_t>
{
public:
  explicit ObjectPropertyChecker(ObjectPropertyGraph* graph) : ValidityChecker(graph) {property_graph_ = graph;}
  ~ObjectPropertyChecker() {}

  size_t check() override;
  void printStatus() override
  {
    ValidityChecker<ObjectPropertyBranch_t>::printStatus("object property", "object properties", graph_vect_.size());
  }
private:
  void checkDisjoint();
  void checkCharacteristics();
  void removeLoops();

  ObjectPropertyGraph* property_graph_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJECTPROPERTYCHECKER_H
