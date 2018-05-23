#ifndef CLASS_DRAWER_H
#define CLASS_DRAWER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/graphical/Drawers/GraphDrawer.h"

class ClassDrawer : public GraphDrawer
{
public:
  ClassDrawer(ClassGraph* graph = nullptr);
  ~ClassDrawer() {}

  void setGraph(ClassGraph* graph) {graph_ = graph; init(); };

  void putInLayers();

private:
  ClassGraph* graph_;
  int createNode(ClassBranch_t* branch, node_t* mother);
  void init();
};

#endif /* CLASS_DRAWER_H */
