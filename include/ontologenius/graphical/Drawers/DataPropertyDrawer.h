#ifndef ONTOLOGENIUS_DATAPROPERTYDRAWER_H
#define ONTOLOGENIUS_DATAPROPERTYDRAWER_H

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/graphical/Drawers/GraphDrawer.h"

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

#endif // ONTOLOGENIUS_DATAPROPERTYDRAWER_H
