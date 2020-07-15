#ifndef ONTOLOGENIUS_VERSIONOR_H
#define ONTOLOGENIUS_VERSIONOR_H

#include <unordered_map>

#include "ontologenius/core/feeder/Version_node.h"
#include "ontologenius/core/feeder/FeedStorage.h"

namespace ontologenius {

class Versionor
{
public:
  Versionor(FeedStorage* storage);
  ~Versionor();

  void activate(bool activated) { activated_ = activated; }

  void insert(feed_t data);
  bool commit(const std::string& id);
  bool checkout(const std::string& id);

  void print() { nodes_["0"]->print(); }
  void exportToXml(const std::string& path);

private:
  bool activated_;
  FeedStorage* storage_;

  std::unordered_map<std::string, Version_node*> nodes_;
  Version_node* current_node_;

  std::vector<Version_node*> getPrevs(Version_node* from_node);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_VERSIONOR_H
