#ifndef ONTOLOGENIUS_BRANCH_H
#define ONTOLOGENIUS_BRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

/*
This file use CRTP (curiously recurring template pattern)
be really carreful of how you use it
*/

namespace ontologenius {

template <typename T>
class Branch_t : public ValuedNode
{
public:
  std::vector<Single_t<T*>> childs_;
  RelationsWithInductions<Single_t<T*>> mothers_;
  std::vector<Single_t<T*>> disjoints_;

  explicit Branch_t(const std::string& value) : ValuedNode(value) {}
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_BRANCH_H
