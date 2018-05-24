#ifndef CLASSCHECKER_H
#define CLASSCHECKER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Checkers/ValidityChecker.h"

class ClassChecker : public ValidityChecker<ClassBranch_t>
{
public:
  ClassChecker(ClassGraph* graph) : ValidityChecker(graph) {class_graph_ = graph;}
  ~ClassChecker() {}

  size_t check();
  void printStatus(){ValidityChecker<ClassBranch_t>::printStatus("class", "classes", graph_.size());}

private:
  void checkDisjoint();

  ClassGraph* class_graph_;
};

#endif
