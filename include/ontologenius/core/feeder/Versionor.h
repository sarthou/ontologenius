#ifndef ONTOLOGENIUS_VERSIONOR_H
#define ONTOLOGENIUS_VERSIONOR_H

#include <unordered_map>

#include "ontologenius/core/feeder/FeedStorage.h"
#include "ontologenius/core/feeder/VersionNode.h"

namespace ontologenius {

  class Versionor
  {
  public:
    explicit Versionor(FeedStorage* storage);
    Versionor(const Versionor& other) = delete;
    ~Versionor();

    Versionor& operator=(const Versionor& other) = delete;

    void activate(bool activated) { activated_ = activated; }

    void insert(feed_t data);
    bool commit(const std::string& id);
    bool checkout(const std::string& id);

    void print() { nodes_["0"]->print(); }
    void exportToXml(const std::string& path);

  private:
    bool activated_;
    FeedStorage* storage_;
    size_t order_;

    std::unordered_map<std::string, VersionNode*> nodes_;
    VersionNode* current_node_;

    std::vector<VersionNode*> getPrevs(VersionNode* from_node);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_VERSIONOR_H
