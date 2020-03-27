#ifndef ONTOLOGENIUS_GRAPHDRAWER_H
#define ONTOLOGENIUS_GRAPHDRAWER_H

#include <string>
#include <vector>
#include <stdint.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/highgui/highgui_c.h>

namespace ontologenius {

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
  std::string value;
  long int layer;
  long int pos;
  bool marker;
  std::vector<node_t*> prev;
  rect_t rect;
  int family;

  node_t(std::string p_value, int p_layer = -1) : value(p_value), layer(p_layer), pos(-1), marker(false) {}
};

class GraphDrawer
{
public:

  void draw(std::string file_name);

protected:
  std::vector<node_t*> roots_nodes;
  std::vector<node_t*> branchs_nodes;

  std::vector<std::vector<node_t*>> layer_nodes;

  IplImage* image;
  bool exist(std::string value);

  void setRect(int layer, int nb_layer, int nb_index, node_t* node);
  void link();

  void putLayer(int layer);
  bool updateOneMarker(int layer);
  bool testEnd();

  cv::Scalar ScalarHSV2BGR(uint8_t H, uint8_t S, uint8_t V);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_GRAPHDRAWER_H
