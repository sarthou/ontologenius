#ifndef OBJECTPROPERTYDRAWER_H
#define OBJECTPROPERTYDRAWER_H

#include "ontoloGenius/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/ontoGraphs/Drawers/GraphDrawer.h"

class ObjectPropertyDrawer : public GraphDrawer
{
public:
  ObjectPropertyDrawer(ObjectPropertyGraph* graph = nullptr);
  ~ObjectPropertyDrawer() {}

  void set_tree(ObjectPropertyGraph* graph) {graph_ = graph; init(); };

  void put_in_layers();

private:
  ObjectPropertyGraph* graph_;
  int create_node(ObjectPropertyBranch_t* branch, node_t* mother);
  void init();
};

#endif /* OBJECTPROPERTYDRAWER_H */
