#ifndef CLASS_DRAWER_H
#define CLASS_DRAWER_H

#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Drawers/GraphDrawer.h"

class ClassDrawer : public GraphDrawer
{
public:
  ClassDrawer(ClassGraph* p_tree = nullptr);
  ~ClassDrawer() {}

  void setGraph(ClassGraph* p_tree) {m_tree = p_tree; init(); };

  void put_in_layers();

private:
  ClassGraph* m_tree;
  int create_node(ClassBranch_t* branch, node_t* mother);
  void init();
};

#endif /* CLASS_DRAWER_H */
