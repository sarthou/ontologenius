#include "ontologenius/graphical/versioning/TreeDrawer.h"

#include <cstddef>
#include <iostream>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <string>

#include "ontologenius/graphical/versioning/TreeReader.h"

#define SPACE 100
#define NODE_RADIUS 20
#define EDGE_RADIUS 30
#define TEXT_WIDTH 750

namespace ontologenius {

  size_t DrawerNode::current_row = 0;
  size_t DrawerNode::current_column = 0;

  TreeDrawer::~TreeDrawer()
  {
    for(auto& node : nodes_)
      delete node;
  }

  void TreeDrawer::draw(const std::string& file_name, Commit* root, bool commit_only)
  {
    createNode(root, commit_only);

    const int width = (int)(DrawerNode::current_column + 1) * SPACE + TEXT_WIDTH;
    const int height = (int)(DrawerNode::current_row + 1) * SPACE;

    std::cout << "image size = " << width << " : " << height << std::endl;
    image_ = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    cvSet(image_, cvScalar(255, 255, 255));

    for(auto& node : nodes_)
    {
      for(auto& next : node->nexts_)
        drawLink(node, next);
      drawNode(node);
      drawNodeText(node);
    }

    drawDelim();

    if(file_name.empty() == false)
    {
      std::cout << "Save image " << file_name << std::endl;

      if((height != 1) && (width != 1))
        cv::imwrite(file_name, cv::cvarrToMat(image_));
    }

    if(image_ != nullptr)
      cvReleaseImage(&image_);
  }

  DrawerNode* TreeDrawer::createNode(Commit* commit, bool commit_only)
  {
    if(commit == nullptr)
      return nullptr;

    DrawerNode* first_node = nullptr;
    DrawerNode* current_node = first_node;

    size_t local_row = getCorrectedRow(commit->order_);
    if(commit_only == false)
      shiftRows(local_row, commit->datas_.size() + 1);
    else
      shiftRows(local_row, 1);

    if(commit_only == false)
      for(auto& data : commit->datas_)
      {
        DrawerNode* data_node = new DrawerNode();
        data_node->row_ = local_row;
        data_node->column_ = DrawerNode::current_column;
        DrawerNode::current_row++;
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

    DrawerNode* commit_node = new DrawerNode();
    commit_node->row_ = local_row;
    commit_node->column_ = DrawerNode::current_column;
    DrawerNode::current_row++;

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
        DrawerNode::current_column++;

      DrawerNode* new_commit_node = createNode(commit->nexts_[i], commit_only);

      new_commit_node->prev_ = current_node;
      current_node->nexts_.push_back(new_commit_node);
    }

    return first_node;
  }

  void TreeDrawer::drawNode(DrawerNode* node)
  {
    const int x = SPACE / 2 + (int)node->column_ * SPACE;
    const int y = SPACE / 2 + (int)node->row_ * SPACE;

    auto color_1 = cvScalar(89, 26, 16);
    auto color_2 = cvScalar(149, 86, 86);
    if(node->is_data_ == false)
    {
      color_1 = cvScalar(32, 20, 122);
      color_2 = cvScalar(114, 102, 204);
    }

    cvCircle(image_, cvPoint(x, y), NODE_RADIUS, color_1, -1, CV_AA, 0);
    cvCircle(image_, cvPoint(x, y), NODE_RADIUS - 2, color_2, -1, CV_AA, 0);
  }

  void TreeDrawer::drawNodeText(DrawerNode* node)
  {
    const int width_base = (int)(DrawerNode::current_column + 1) * SPACE;
    const int y = SPACE / 2 + (int)node->row_ * SPACE;

    auto color = cvScalar(89, 26, 16);
    if(node->is_data_ == false)
      color = cvScalar(32, 20, 122);

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1, 1, 0, 2);
    cvPutText(image_, node->text_.c_str(), cvPoint(width_base + SPACE / 2, y + NODE_RADIUS), &font,
              color);
  }

  void TreeDrawer::drawLink(DrawerNode* node_from, DrawerNode* node_to)
  {
    const int x_from = SPACE / 2 + (int)node_from->column_ * SPACE;
    const int y_from = SPACE / 2 + (int)node_from->row_ * SPACE;

    const int x_to = SPACE / 2 + (int)node_to->column_ * SPACE;
    const int y_to = SPACE / 2 + (int)node_to->row_ * SPACE;

    if(node_from->column_ == node_to->column_)
    {
      cvLine(image_, cvPoint(x_from, y_from + NODE_RADIUS / 2),
             cvPoint(x_to, y_to - NODE_RADIUS / 2),
             cvScalar(50, 50, 50), 2);
    }
    else
    {
      drawElipseRight(x_from, y_from + SPACE / 2);
      static_assert(SPACE - EDGE_RADIUS * 2 > 0, "TreeDrawer has bad SPACE and EDGE_RADIUS values");
      cvLine(image_, cvPoint(x_from + EDGE_RADIUS, y_from + SPACE / 2),
             cvPoint(x_to - EDGE_RADIUS, y_from + SPACE / 2),
             cvScalar(50, 50, 50), 2);
      drawElipseBottom(x_to, y_from + SPACE / 2);
      cvLine(image_, cvPoint(x_to, y_from + SPACE / 2 + EDGE_RADIUS),
             cvPoint(x_to, y_to - NODE_RADIUS / 2),
             cvScalar(50, 50, 50), 2);
    }
  }

  void TreeDrawer::drawElipseRight(size_t x, size_t y)
  {
    cvEllipse(image_, cvPoint((int)x + EDGE_RADIUS, (int)y - EDGE_RADIUS), cvSize(EDGE_RADIUS, EDGE_RADIUS), 180, 0, -90, cvScalar(50, 50, 50), 2);
  }

  void TreeDrawer::drawElipseBottom(size_t x, size_t y)
  {
    cvEllipse(image_, cvPoint((int)x - EDGE_RADIUS, (int)y + EDGE_RADIUS), cvSize(EDGE_RADIUS, EDGE_RADIUS), -90, 0, 90, cvScalar(50, 50, 50), 2);
  }

  void TreeDrawer::drawDelim()
  {
    const int width = (int)(DrawerNode::current_column + 1) * SPACE;
    const int height = (int)(DrawerNode::current_row + 1) * SPACE;
    cvLine(image_, cvPoint(width, 0),
           cvPoint(width, height),
           cvScalar(50, 50, 50), 3);
  }

  size_t TreeDrawer::getCorrectedRow(int id)
  {
    size_t min_delta = -1;
    size_t row = DrawerNode::current_row;

    for(auto& node : commit_nodes_)
    {
      const int delta = id - node.first;
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
