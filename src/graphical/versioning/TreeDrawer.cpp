#include "ontologenius/graphical/versioning/TreeDrawer.h"

#include <opencv2/imgproc/imgproc.hpp>

#define SPACE 100
#define NODE_RADIUS 20
#define EDGE_RADIUS 30
#define TEXT_WIDTH 750

namespace ontologenius {

size_t Node_t::current_row = 0;
size_t Node_t::current_column = 0;

TreeDrawer::~TreeDrawer()
{
  for(auto& node : nodes_)
    delete node;
}

void TreeDrawer::draw(const std::string& file_name, commit_t* root, bool commit_only)
{
  createNode(root, commit_only);

  size_t width = (Node_t::current_column + 1) * SPACE + TEXT_WIDTH;
  size_t height = (Node_t::current_row + 1) * SPACE;

  std::cout << "image size = " << width << " : " << height << std::endl;
  image_ = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  cvSet(image_, cvScalar(255,255,255));

  for(auto& node : nodes_)
  {
    for(auto& next : node->nexts_)
      drawLink(node, next);
    drawNode(node);
    drawNodeText(node);
  }

  drawDelim();

  if(file_name != "")
  {
    std::cout << "Save image " << file_name << std::endl;

    if((height != 1) && (width != 1))
      cv::imwrite(file_name.c_str(), cv::cvarrToMat(image_));
  }

  if(image_ != nullptr)
    cvReleaseImage(&image_);
}

Node_t* TreeDrawer::createNode(commit_t* commit, bool commit_only)
{
  if(commit == nullptr)
    return nullptr;

  Node_t* first_node = nullptr;
  Node_t* current_node = first_node;

  size_t local_row = getCorrectedRow(commit->order_);
  if(commit_only == false)
    shiftRows(local_row, commit->datas_.size() + 1);
  else
    shiftRows(local_row, 1);

  if(commit_only == false)
    for(auto& data : commit->datas_)
    {
      Node_t* data_node = new Node_t();
      data_node->row_ = local_row;
      data_node->column_ = Node_t::current_column;
      Node_t::current_row++;
      local_row++;

      data_node->is_data_ = true;
      data_node->text_ = data;
      nodes_.push_back(data_node);

      if(current_node != nullptr)
      {
        data_node->prev_ = current_node;
        current_node->nexts_.push_back(data_node);
      }

      if(first_node == nullptr)
        first_node = data_node;

      current_node = data_node;
    }

  Node_t* commit_node = new Node_t();
  commit_node->row_ = local_row;
  commit_node->column_ = Node_t::current_column;
  Node_t::current_row++;

  commit_node->is_data_ = false;
  commit_node->text_ = "commit : " + commit->id_;
  nodes_.push_back(commit_node);
  commit_nodes_[commit->order_] = commit_node;

  if(current_node != nullptr)
  {
    commit_node->prev_ = current_node;
    current_node->nexts_.push_back(commit_node);
  }

  current_node = commit_node;
  if(first_node == nullptr)
    first_node = commit_node;

  for(size_t i = 0; i < commit->nexts_.size(); i++)
  {
    if(i > 0)
      Node_t::current_column++;

    Node_t* new_commit_node = createNode(commit->nexts_[i], commit_only);

    new_commit_node->prev_ = current_node;
    current_node->nexts_.push_back(new_commit_node);
  }

  return first_node;
}

void TreeDrawer::drawNode(Node_t* node)
{
  size_t x = SPACE / 2 + node->column_ * SPACE;
  size_t y = SPACE / 2 + node->row_ * SPACE;

  auto color_1 = cvScalar(89, 26, 16);
  auto color_2 = cvScalar(149,86,86);
  if(node->is_data_ == false)
  {
    color_1 = cvScalar(32, 20, 122);
    color_2 = cvScalar(114,102,204);
  }

  cvCircle(image_, cvPoint(x,y), NODE_RADIUS, color_1, -1, CV_AA, 0);
  cvCircle(image_, cvPoint(x,y), NODE_RADIUS - 2, color_2, -1, CV_AA, 0);
}

void TreeDrawer::drawNodeText(Node_t* node)
{
  size_t width_base = (Node_t::current_column + 1) * SPACE;
  size_t y = SPACE / 2 + node->row_ * SPACE;

  auto color = cvScalar(89, 26, 16);
  if(node->is_data_ == false)
    color = cvScalar(32, 20, 122);

  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1, 1, 0, 2);
  cvPutText(image_, node->text_.c_str(), cvPoint(width_base + SPACE/2,y+NODE_RADIUS), &font,
            color);
}

void TreeDrawer::drawLink(Node_t* node_from, Node_t* node_to)
{
  size_t x_from = SPACE / 2 + node_from->column_ * SPACE;
  size_t y_from = SPACE / 2 + node_from->row_ * SPACE;

  size_t x_to = SPACE / 2 + node_to->column_ * SPACE;
  size_t y_to = SPACE / 2 + node_to->row_ * SPACE;

  if(node_from->column_ == node_to->column_)
  {
    cvLine(image_, cvPoint(x_from, y_from+NODE_RADIUS/2),
                   cvPoint(x_to, y_to-NODE_RADIUS/2),
                   cvScalar(50,50,50), 2);
  }
  else
  {
    drawElipseRight(x_from, y_from+SPACE/2);
    int length = SPACE - EDGE_RADIUS*2;
    if(length > 0)
      cvLine(image_, cvPoint(x_from + EDGE_RADIUS, y_from + SPACE/2),
                     cvPoint(x_to - EDGE_RADIUS, y_from + SPACE/2),
                     cvScalar(50,50,50), 2);
    drawElipseBottom(x_to, y_from+SPACE/2);
    cvLine(image_, cvPoint(x_to, y_from+SPACE/2 + EDGE_RADIUS),
                   cvPoint(x_to, y_to-NODE_RADIUS/2),
                   cvScalar(50,50,50), 2);
  }
}

void TreeDrawer::drawElipseRight(size_t x, size_t y)
{
  cvEllipse(image_, cvPoint(x+EDGE_RADIUS,y-EDGE_RADIUS), cvSize(EDGE_RADIUS, EDGE_RADIUS), 180, 0, -90, cvScalar(50,50,50), 2);
}

void TreeDrawer::drawElipseBottom(size_t x, size_t y)
{
  cvEllipse(image_, cvPoint(x-EDGE_RADIUS,y+EDGE_RADIUS), cvSize(EDGE_RADIUS, EDGE_RADIUS), -90, 0, 90, cvScalar(50,50,50), 2);
}

void TreeDrawer::drawDelim()
{
  size_t width = (Node_t::current_column + 1) * SPACE;
  size_t height = (Node_t::current_row + 1) * SPACE;
  cvLine(image_, cvPoint(width, 0),
                 cvPoint(width, height),
                 cvScalar(50,50,50), 3);
}

size_t TreeDrawer::getCorrectedRow(int id)
{
  size_t min_delta = -1;
  size_t row = Node_t::current_row;

  for(auto& node : commit_nodes_)
  {
    int delta = id - node.first;
    if(delta > 0)
    {
      if((size_t)delta < min_delta)
      {
        min_delta = delta;
        row = node.second->row_ + 1;
      }
    }
  }

  return row;
}

void TreeDrawer::shiftRows(size_t row, size_t shift)
{
  for(auto& node : nodes_)
  {
    if(node->row_ >= row)
      node->row_ += shift;
  }
}

} // namespace ontologenius
