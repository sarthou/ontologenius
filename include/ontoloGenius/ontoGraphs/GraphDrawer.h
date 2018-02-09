#include <string>
#include <vector>
#include <stdint.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/highgui/highgui_c.h>

#include "ontoloGenius/ontoGraphs/ClassGraph.h"

#ifndef TREE_DRAWER_H
#define TREE_DRAWER_H

struct rect_t
{
  long int x,y;
  long int width, height;

  rect_t(long int p_x = 0, long int p_y = 0, long int p_width = 0, long int p_height = 0) : x(p_x), y(p_y), width(p_width), height(p_height) {}

  long int x_middle_top() { return x + width/2; }
  long int y_middle_top() { return y; }
  long int x_middle_bot() { return x + width/2; }
  long int y_middle_bot() { return y + height; }
};

struct node_t
{
  long int layer;
  long int pos;
  bool marker;
  std::vector<node_t*> prev;
  std::string value;
  rect_t rect;
  int family;

  node_t(std::string p_value, int p_layer = -1) : value(p_value), layer(p_layer), pos(-1), marker(false) {}
};

class GraphDrawer
{
public:
  GraphDrawer(ClassGraph* p_tree = nullptr);
  ~GraphDrawer() {}

  void set_tree(ClassGraph* p_tree) {m_tree = p_tree; init(); };

  void put_in_layers();
  void draw(std::string file_name);

private:
  std::vector<node_t*> roots_nodes;
  std::vector<node_t*> branchs_nodes;

  std::vector<std::vector<node_t*>> layer_nodes;

  IplImage* image;
  ClassGraph* m_tree;
  bool exist(std::string value);
  int create_node(ClassBranch_t* branch, node_t* mother);
  void init();

  void set_rect(int layer, int nb_layer, int nb_index, node_t* node);
  void link();

  void put_layer(int layer);
  bool update_one_marker(int layer);
  bool test_end();

  cv::Scalar ScalarHSV2BGR(uint8_t H, uint8_t S, uint8_t V);
};

#endif /* TREE_DRAWER_H */
