#ifndef CLASSCHECKER_H
#define CLASSCHECKER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Checkers/ValidityChecker.h"

namespace ontologenius {

class ClassChecker : public ValidityChecker<ClassBranch_t>
{
public:
  ClassChecker(ClassGraph* graph) : ValidityChecker(graph) {class_graph_ = graph;}
  ~ClassChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<ClassBranch_t>::printStatus("class", "classes", graph_vect_.size());}

private:
  void checkDisjoint();

  void checkObectPropertyDomain();
  void checkObectPropertyRange();

  void checkDataPropertyDomain();
  void checkDataPropertyRange();

  ClassGraph* class_graph_;
};

} // namespace ontologenius

#endif
