#ifndef CLASSCHECKER_H
#define CLASSCHECKER_H

#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Checkers/ValidityChecker.h"

class ClassChecker : public ValidityChecker<ClassBranch_t>
{
public:
  ClassChecker(ClassGraph* graph) : ValidityChecker(graph) {class_graph_ = graph;}
  ~ClassChecker() {}

  size_t check();
private:
  size_t checkDisjoint();

  ClassGraph* class_graph_;
};

#endif
