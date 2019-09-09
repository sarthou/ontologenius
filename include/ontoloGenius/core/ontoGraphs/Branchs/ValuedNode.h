#ifndef ONTOLOGENIUS_VALUEDNODE_H
#define ONTOLOGENIUS_VALUEDNODE_H

#include <string>
#include <vector>
#include <map>

#include "ontoloGenius/core/ontoGraphs/Branchs/WordTable.h"

namespace ontologenius {

class UpdatableNode
{
public:
  unsigned int nb_updates_;
  bool updated_;
  std::map<std::string, std::vector<std::string>> flags_;
  UpdatableNode() {updated_ = true; nb_updates_ = 0; }
};

class ValuedNodeData
{
public:
  std::map<std::string, std::vector<std::string>> dictionary_;
  std::map<std::string, std::vector<std::string>> muted_dictionary_;
};

class ValuedNode : public UpdatableNode, public ValuedNodeData
{
public:
  ValuedNode(const std::string& value) : UpdatableNode() {index_ = table_.add(value); }

  uint32_t get() {return index_; }
  std::string value() {return table_[index_]; }

  static WordTable table_;
private:

  uint32_t index_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_VALUEDNODE_H
