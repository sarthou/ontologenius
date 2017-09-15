#include "ontoloGenius/tree_drawer.h"
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace std;

#define MARKER_WIDTH      350
#define MIN_WIDTH_SPACE   50
#define MARKER_HEIGHT     100
#define MIN_HEIGHT_SPACE  1000

tree_drawer::tree_drawer(tree* p_tree)
{
  m_tree = p_tree;
  init();
}

void tree_drawer::put_in_layers()
{
  if((m_tree != nullptr) && (m_tree->roots.size() != 0))
  {
    //init markers
    for(unsigned long int i = 0; i < branchs_nodes.size(); i++)
      branchs_nodes[i]->marker = false;

    for(unsigned long int i = 0; i < roots_nodes.size(); i++)
    {
      roots_nodes[i]->marker = false;
      roots_nodes[i]->pos = i;
    }

    int layer = 0;

    while(!test_end())
    {
      layer_nodes.push_back(vector<node_t*>());
      put_layer(layer);
      layer++;
    }
    layer_nodes.pop_back();
  }
}

void tree_drawer::put_layer(int layer)
{
  int pos = 0;
  bool had_update = true;
  while(had_update)
  {
    had_update = update_one_marker(layer);

    for(unsigned long int i = 0; i < branchs_nodes.size(); i++)
    {
      bool previous_done = true;
      for(unsigned long int pre = 0; pre < branchs_nodes[i]->prev.size(); pre++)
        if(branchs_nodes[i]->prev[pre]->marker == false)
          previous_done = false;

      if(previous_done && (branchs_nodes[i]->layer == -1))
      {
        branchs_nodes[i]->layer = layer+1;
        branchs_nodes[i]->pos = pos;
        pos++;
        layer_nodes[layer].push_back(branchs_nodes[i]);
      }
    } // end for
  } // end while
}

bool tree_drawer::update_one_marker(int layer)
{
  for(unsigned long int i = 0; i < roots_nodes.size(); i++)
    if((roots_nodes[i]->layer == layer) && (roots_nodes[i]->marker == false))
    {
      roots_nodes[i]->marker = true;
      return true;
    }

  for(unsigned long int layer_i = 0; layer_i < layer_nodes.size(); layer_i++)
    for(unsigned long int i = 0; i < layer_nodes[layer_i].size(); i++)
      if((layer_nodes[layer_i][i]->layer == layer) && (layer_nodes[layer_i][i]->marker == false))
      {
        layer_nodes[layer_i][i]->marker = true;
        return true;
      }

  return false;
}

bool tree_drawer::test_end()
{
  bool end = true;

  for(unsigned long int i = 0; i < branchs_nodes.size(); i++)
    if(branchs_nodes[i]->marker == false)
      end = false;

  return end;
}

void tree_drawer::draw(string file_name)
{
  long int height = (layer_nodes.size() /*+1*/)*(MARKER_HEIGHT + MIN_HEIGHT_SPACE) + 1;

  long int width = roots_nodes.size();
  for(unsigned long int i  = 0; i < layer_nodes.size(); i++)
      if(layer_nodes[i].size() > width)
        width = layer_nodes[i].size();
  width = width*(MARKER_WIDTH + MIN_WIDTH_SPACE) + 1;

  cout << height << " : " << width << endl;

  image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  cvSet(image, cvScalar(255,255,255));

  for(unsigned long int i = 0; i < roots_nodes.size(); i++)
    set_rect(0, layer_nodes.size() + 1, roots_nodes.size(), roots_nodes[i]);

  for(unsigned long int layer = 0; layer < layer_nodes.size(); layer++)
    for(unsigned long int i = 0; i < layer_nodes[layer].size(); i++)
    set_rect(layer+1, layer_nodes.size() + 1, layer_nodes[layer].size(), layer_nodes[layer][i]);

  link();

  if(file_name == "")
    file_name = "ontology.png";
  cvSaveImage(file_name.c_str(), image);
}

void tree_drawer::set_rect(int layer, int nb_layer, int nb_index, node_t* node)
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
            ScalarHSV2BGR(node->family, 255 - node->family/2, 255 - node->family/2),
            -1, 8, 0);
  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.8, 1.0, 0, 3);
  cvPutText(image, node->value.c_str(), cvPoint(x+5,y+MARKER_HEIGHT/2), &font,
            ScalarHSV2BGR(255, node->family, node->family));
}

void tree_drawer::link()
{
  for(unsigned long int layer = 0; layer < layer_nodes.size(); layer++)
    for(unsigned long int i = 0; i < layer_nodes[layer].size(); i++)
      for(unsigned long int pre = 0; pre < layer_nodes[layer][i]->prev.size(); pre++)
      {
        cvLine(image, cvPoint(layer_nodes[layer][i]->rect.x_middle_top(),
                              layer_nodes[layer][i]->rect.y_middle_top()),
        cvPoint(layer_nodes[layer][i]->prev[pre]->rect.x_middle_bot(),
                layer_nodes[layer][i]->prev[pre]->rect.y_middle_bot()),
        cvScalar(0), 2);
      }
}

bool tree_drawer::exist(string value)
{
  for(unsigned long int i = 0; i < branchs_nodes.size(); i++)
    if(branchs_nodes[i]->value == value)
      return true;

  return false;
}

int tree_drawer::create_node(branch_t* branch, node_t* mother)
{
  int family = branch->family;
  if(!exist(branch->value))
  {
    node_t* node = new node_t(branch->value);
    branchs_nodes.push_back(node);
    node->prev.push_back(mother);
    node->family = branch->family;
    for(unsigned long int i = 0; i < branch->childs.size(); i++)
      family += create_node(branch->childs[i], node);

    family = family / (branch->childs.size() + 1);
  }
  else
  {
    for(unsigned long int i = 0; i < branchs_nodes.size(); i++)
      if(branchs_nodes[i]->value == branch->value)
        branchs_nodes[i]->prev.push_back(mother);
  }

}

void tree_drawer::init()
{
  vector<node_t*> single;
  vector<node_t*> couple;
  if(m_tree != nullptr)
  {
    for(unsigned long int i = 0; i < m_tree->roots.size(); i++)
    {
      node_t* node = new node_t(m_tree->roots[i]->value, 0);
      //roots_nodes.push_back(node);
      node->family = m_tree->roots[i]->family;
      int family = m_tree->roots[i]->family;

      for(unsigned long int branch = 0; branch < m_tree->roots[i]->childs.size(); branch++)
        family += create_node(m_tree->roots[i]->childs[branch], node);

      family = family / (m_tree->roots[i]->childs.size() + 1);
      if(family == node->family)
        single.push_back(node);
      else
        couple.push_back(node);
    }

    int middle = single.size()/2;
    for(unsigned long int i = 0; i < middle; i++)
      roots_nodes.push_back(single[i]);

    for(unsigned long int i = 0; i < couple.size(); i++)
      roots_nodes.push_back(couple[i]);

    for(unsigned long int i = middle; i < single.size(); i++)
      roots_nodes.push_back(single[i]);
  }
}

cv::Scalar tree_drawer::ScalarHSV2BGR(uint8_t H, uint8_t S, uint8_t V)
{
    cv::Mat rgb;
    cv::Mat hsv(1,1, CV_8UC3, cv::Scalar(H,S,V));
    cvtColor(hsv, rgb, CV_HSV2BGR);
    return cv::Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
}
