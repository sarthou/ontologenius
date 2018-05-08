#include <string>
#include <vector>
#include <stdint.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/highgui/highgui_c.h>

#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Drawers/GraphDrawer.h"

#ifndef CLASS_DRAWER_H
#define CLASS_DRAWER_H

class ClassDrawer : public GraphDrawer
{
public:
  ClassDrawer(ClassGraph* p_tree = nullptr);
  ~ClassDrawer() {}

  void set_tree(ClassGraph* p_tree) {m_tree = p_tree; init(); };

  void put_in_layers();

private:
  ClassGraph* m_tree;
  int create_node(ClassBranch_t* branch, node_t* mother);
  void init();
};

#endif /* CLASS_DRAWER_H */
