#ifndef ONTOLOGENIUS_VALUEDNODE_H
#define ONTOLOGENIUS_VALUEDNODE_H

#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"

namespace ontologenius {

class UpdatableNode
{
public:
  unsigned int nb_updates_;
  bool updated_;
  std::map<std::string, std::vector<std::string>> flags_;
  UpdatableNode() {updated_ = true; nb_updates_ = 0; }
};

class Dictionary
{
public:
  std::map<std::string, std::vector<std::string>> spoken_;
  std::map<std::string, std::vector<std::string>> muted_;
};

class ValuedNode : public UpdatableNode
{
public:
  ValuedNode(const std::string& value) : UpdatableNode() {index_ = table_.add(value); }

  uint32_t get() {return index_; }
  std::string value() {return table_[index_]; }

  static WordTable table_;

  Dictionary dictionary_;
  Dictionary steady_dictionary_;

  template<typename C>
  inline void conditionalPushBack(std::vector<C>& vect, const C& data)
  {
    if(std::find(vect.begin(), vect.end(), data) == vect.end())
      vect.push_back(data);
  }

private:

  uint32_t index_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_VALUEDNODE_H
