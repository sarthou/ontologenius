#ifndef ONTOLOGENIUS_CLASSDRAWER_H
#define ONTOLOGENIUS_CLASSDRAWER_H

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/graphical/Drawers/GraphDrawer.h"

namespace ontologenius {

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

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSDRAWER_H
