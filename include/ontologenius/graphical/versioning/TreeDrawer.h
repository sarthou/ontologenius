#ifndef ONTOLOGENIUS_TREEDRAWER_H
#define ONTOLOGENIUS_TREEDRAWER_H

#include <map>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgcodecs.hpp>

#include "ontologenius/graphical/versioning/TreeReader.h"

namespace ontologenius {

  class DrawerNode
  {
  public:
    size_t row_;
    size_t column_;

    static size_t current_row;
    static size_t current_column;

    bool is_data_;
    std::string text_;

    DrawerNode* prev_;
    std::vector<DrawerNode*> nexts_;
  };

  class TreeDrawer
  {
  public:
    TreeDrawer() : image_(nullptr) {}
    ~TreeDrawer();

    void draw(const std::string& file_name, Commit* root, bool commit_only = false);

  private:
    std::vector<DrawerNode*> nodes_;
    std::map<int, DrawerNode*> commit_nodes_;
    IplImage* image_;

    DrawerNode* createNode(Commit* commit, bool commit_only);

    void drawNode(DrawerNode* node);
    void drawNodeText(DrawerNode* node);
    void drawLink(DrawerNode* node_from, DrawerNode* node_to);
    void drawElipseRight(size_t x, size_t y);
    void drawElipseBottom(size_t x, size_t y);

    void drawDelim();

    size_t getCorrectedRow(int id);
    void shiftRows(size_t row, size_t shift);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_TREEDRAWER_H
