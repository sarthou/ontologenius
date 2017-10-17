#include <string>
#include <vector>
#include <stdint.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/highgui/highgui_c.h>

#include "ontoloGenius/TreeObject.h"

#ifndef TREE_DRAWER_H
#define TREE_DRAWER_H

using namespace std;

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
  vector<node_t*> prev;
  string value;
  rect_t rect;
  int family;

  node_t(string p_value, int p_layer = -1) : value(p_value), layer(p_layer), pos(-1), marker(false) {}
};

class TreeDrawer
{
public:
  TreeDrawer(TreeObject* p_tree = nullptr);
  ~TreeDrawer() {}

  void set_tree(TreeObject* p_tree) {m_tree = p_tree; init(); };

  void put_in_layers();
  void draw(string file_name);

private:
  vector<node_t*> roots_nodes;
  vector<node_t*> branchs_nodes;

  vector<vector<node_t*>> layer_nodes;

  IplImage* image;
  TreeObject* m_tree;
  bool exist(string value);
  int create_node(Branch_t* branch, node_t* mother);
  void init();

  void set_rect(int layer, int nb_layer, int nb_index, node_t* node);
  void link();

  void put_layer(int layer);
  bool update_one_marker(int layer);
  bool test_end();

  cv::Scalar ScalarHSV2BGR(uint8_t H, uint8_t S, uint8_t V);
};

#endif /* TREE_DRAWER_H */
