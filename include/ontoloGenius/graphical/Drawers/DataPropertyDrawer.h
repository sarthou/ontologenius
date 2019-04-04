#ifndef DATAPROPERTYDRAWER_H
#define DATAPROPERTYDRAWER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontoloGenius/graphical/Drawers/GraphDrawer.h"

namespace ontologenius {

class DataPropertyDrawer : public GraphDrawer
{
public:
  DataPropertyDrawer(DataPropertyGraph* graph = nullptr);
  ~DataPropertyDrawer() {}

  void setGraph(DataPropertyGraph* graph) {graph_ = graph; init(); };

  void putInLayers();

private:
  DataPropertyGraph* graph_;
  int createNode(DataPropertyBranch_t* branch, node_t* mother);
  void init();
};

} // namespace ontologenius

#endif /* DATAPROPERTYDRAWER_H */
