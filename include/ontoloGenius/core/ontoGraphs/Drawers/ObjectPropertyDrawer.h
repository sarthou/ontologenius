#ifndef OBJECTPROPERTYDRAWER_H
#define OBJECTPROPERTYDRAWER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Drawers/GraphDrawer.h"

class ObjectPropertyDrawer : public GraphDrawer
{
public:
  ObjectPropertyDrawer(ObjectPropertyGraph* graph = nullptr);
  ~ObjectPropertyDrawer() {}

  void setGraph(ObjectPropertyGraph* graph) {graph_ = graph; init(); };

  void putInLayers();

private:
  ObjectPropertyGraph* graph_;
  int createNode(ObjectPropertyBranch_t* branch, node_t* mother);
  void init();
};

#endif /* OBJECTPROPERTYDRAWER_H */
