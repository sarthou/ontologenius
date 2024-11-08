#ifndef ONTOLOGENIUS_VERSIONNODE_H
#define ONTOLOGENIUS_VERSIONNODE_H

#include <algorithm>
#include <iostream>
#include <vector>

#include "ontologenius/core/feeder/FeedStorage.h"

namespace ontologenius {

  class VersionNode
  {
  public:
    explicit VersionNode(size_t order, VersionNode* prev = nullptr);
    VersionNode(size_t order, const std::string& id);

    void insert(const Feed_t& data) { datas_.push_back(data); }
    void setId(const std::string& id) { id_ = id; }
    void addNext(VersionNode* next) { nexts_.push_back(next); }

    std::string getId() const { return id_; }
    bool defined() const { return id_.empty() == false; }
    VersionNode* getPrev() const { return prev_; }
    std::vector<VersionNode*> getNexts() const { return nexts_; }

    std::vector<Feed_t> getDatasDirect();
    std::vector<Feed_t> getDatasInvert();

    void appendDatasDirect(std::vector<Feed_t>& datas);
    void appendDatasInvert(std::vector<Feed_t>& datas);

    void unlinkFromPrev();

    void print(int level = 0);
    std::string toXml(int level = 0);

  private:
    std::string id_;
    size_t order_id_;

    VersionNode* prev_;
    std::vector<VersionNode*> nexts_;

    std::vector<Feed_t> datas_;

    std::string getSpaces(int nb, const std::string& symbol = "  ");
    std::string dataToXml(const Feed_t& data) const;
    std::string orderIdToXml() const;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_VERSIONNODE_H
