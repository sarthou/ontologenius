#ifndef OBJECTPROPERTYDRAWER_H
#define OBJECTPROPERTYDRAWER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/graphical/Drawers/GraphDrawer.h"

namespace ontologenius {

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

} // namespace ontologenius

#endif /* OBJECTPROPERTYDRAWER_H */
