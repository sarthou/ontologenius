#ifndef ONTOLOGENIUS_VERSIONNODE_H
#define ONTOLOGENIUS_VERSIONNODE_H

#include <vector>
#include <algorithm>
#include <iostream>

#include "ontologenius/core/feeder/FeedStorage.h"

namespace ontologenius {

class Version_node
{
public:
  Version_node(Version_node* prev = nullptr);
  Version_node(const std::string& id);

  void insert(const feed_t& data) { datas_.push_back(data); }
  void setId(const std::string& id) { id_ = id; }
  void addNext(Version_node* next) { nexts_.push_back(next); }

  std::string getId() { return id_; }
  bool defined() { return id_ != ""; }
  Version_node* getPrev() { return prev_; }
  std::vector<Version_node*> getNexts() { return nexts_; }

  std::vector<feed_t> getDatasDirect();
  std::vector<feed_t> getDatasInvert();

  void appendDatasDirect(std::vector<feed_t>& datas);
  void appendDatasInvert(std::vector<feed_t>& datas);

  void unlinkFromPrev();

  void print(int level = 0);
  std::string toXml(int level = 0);

private:
  std::string id_;
  size_t order_id_;
  static size_t global_order_id_;

  Version_node* prev_;
  std::vector<Version_node*> nexts_;

  std::vector<feed_t> datas_;

  std::string getSpaces(int nb, const std::string& symbol = "  ");
  std::string dataToXml(const feed_t& data);
  std::string orderIdToXml();
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_VERSIONNODE_H
