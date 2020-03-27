#include "ontologenius/graphical/Drawers/GraphDrawer.h"

#include <opencv2/imgproc/imgproc.hpp>

#define MARKER_WIDTH      350
#define MIN_WIDTH_SPACE   50
#define MARKER_HEIGHT     100
#define MIN_HEIGHT_SPACE  1000

namespace ontologenius {

void GraphDrawer::putLayer(int layer)
{
  int pos = 0;
  bool had_update = true;
  while(had_update)
  {
    had_update = updateOneMarker(layer);

    for(auto& branchs_node : branchs_nodes)
    {
      bool previous_done = true;
      for(auto& prev : branchs_node->prev)
        if(prev->marker == false)
          previous_done = false;

      if(previous_done && (branchs_node->layer == -1))
      {
        branchs_node->layer = layer+1;
        branchs_node->pos = pos;
        pos++;
        layer_nodes[layer].push_back(branchs_node);
      }
    } // end for
  } // end while
}

bool GraphDrawer::updateOneMarker(int layer)
{
  for(auto& roots_node : roots_nodes)
    if((roots_node->layer == layer) && (roots_node->marker == false))
    {
      roots_node->marker = true;
      return true;
    }

  for(auto& layer_i : layer_nodes)
    for(auto& layer_node: layer_i)
      if((layer_node->layer == layer) && (layer_node->marker == false))
      {
        layer_node->marker = true;
        return true;
      }

  return false;
}

bool GraphDrawer::testEnd()
{
  bool end = true;

  for(auto& branchs_node : branchs_nodes)
    if(branchs_node->marker == false)
      end = false;

  return end;
}

void GraphDrawer::draw(std::string file_name)
{
  size_t height = (layer_nodes.size())*(MARKER_HEIGHT + MIN_HEIGHT_SPACE) + 1;

  size_t width = roots_nodes.size();
  for(auto& layer_node : layer_nodes)
      if(layer_node.size() > width)
        width = layer_node.size();
  width = width*(MARKER_WIDTH + MIN_WIDTH_SPACE) + 1;

  image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  cvSet(image, cvScalar(255,255,255));

  for(auto& roots_node : roots_nodes)
    setRect(0, layer_nodes.size() + 1, roots_nodes.size(), roots_node);

  for(size_t layer = 0; layer < layer_nodes.size(); layer++)
    for(auto& layer_node : layer_nodes[layer])
      setRect(layer+1, layer_nodes.size() + 1, layer_nodes[layer].size(), layer_node);

  link();

  if(file_name == "")
    file_name = "ontology.png";

  if((height != 1) && (width != 1))
    cvSaveImage(file_name.c_str(), image);
}

void GraphDrawer::setRect(int layer, int nb_layer, int nb_index, node_t* node)
{
  long int X = cvGetSize(image).width;
  long int Y = cvGetSize(image).height;

  long int x = (X/nb_index - MARKER_WIDTH) / 2;
  x = x + node->pos * (MARKER_WIDTH + (X/nb_index - MARKER_WIDTH));
  long int y = (Y/(nb_layer + 1) - MARKER_HEIGHT) / 2;
  y = y + layer * (MARKER_HEIGHT + (Y/(nb_layer + 1) - MARKER_HEIGHT));

  node->rect = rect_t(x, y, MARKER_WIDTH, MARKER_HEIGHT);

  cvRectangle(image, cvPoint(x,y),
            cvPoint(x+MARKER_WIDTH, y+MARKER_HEIGHT),
            ScalarHSV2BGR(node->family, 200, 255),
            -1, 8, 0);
  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1.1, 1.5, 0, 3);
  cvPutText(image, node->value.c_str(), cvPoint(x+5,y+MARKER_HEIGHT/2), &font,
            ScalarHSV2BGR(255, 255, 0));
}

void GraphDrawer::link()
{
  for(size_t layer = 0; layer < layer_nodes.size(); layer++)
    for(size_t i = 0; i < layer_nodes[layer].size(); i++)
      for(size_t pre = 0; pre < layer_nodes[layer][i]->prev.size(); pre++)
      {
        cvLine(image, cvPoint(layer_nodes[layer][i]->rect.x_middle_top(),
                              layer_nodes[layer][i]->rect.y_middle_top()),
        cvPoint(layer_nodes[layer][i]->prev[pre]->rect.x_middle_bot(),
                layer_nodes[layer][i]->prev[pre]->rect.y_middle_bot()),
        cvScalar(0), 2);
      }
}

bool GraphDrawer::exist(std::string value)
{
  for(size_t i = 0; i < branchs_nodes.size(); i++)
    if(branchs_nodes[i]->value == value)
      return true;

  return false;
}

cv::Scalar GraphDrawer::ScalarHSV2BGR(uint8_t H, uint8_t S, uint8_t V)
{
    cv::Mat rgb;
    cv::Mat hsv(1,1, CV_8UC3, cv::Scalar(H,S,V));
    cvtColor(hsv, rgb, CV_HSV2BGR);
    return cv::Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
}

} // namespace ontologenius
