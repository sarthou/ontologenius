#ifndef ONTOLOGENIUS_TREEDRAWER_H
#define ONTOLOGENIUS_TREEDRAWER_H

#include "ontologenius/graphical/versioning/TreeReader.h"

#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/highgui/highgui_c.h>

#include <map>

namespace ontologenius {

class Node_t
{
public:
  size_t row_;
  size_t column_;

  static size_t current_row;
  static size_t current_column;

  bool is_data_;
  std::string text_;

  Node_t* prev_;
  std::vector<Node_t*> nexts_;
};

class TreeDrawer
{
public:
  ~TreeDrawer();

  void draw(const std::string& file_name, commit_t* root, bool commit_only = false);

private:
  std::vector<Node_t*> nodes_;
  std::map<int, Node_t*> commit_nodes_;
  IplImage* image_;

  Node_t* createNode(commit_t* commit, bool commit_only);

  void drawNode(Node_t* node);
  void drawNodeText(Node_t* node);
  void drawLink(Node_t* node_from, Node_t* node_to);
  void drawElipseRight(size_t x, size_t y);
  void drawElipseBottom(size_t x, size_t y);

  void drawDelim();

  size_t getCorrectedRow(int id);
  void shiftRows(size_t row, size_t shift);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_TREEDRAWER_H
