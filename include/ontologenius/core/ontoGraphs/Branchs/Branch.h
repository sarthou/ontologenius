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
  std::vector<Single_t<T*>> mothers_;

  uint8_t family;
  uint8_t nb_mothers_;

  Branch_t(std::string value) : ValuedNode(value), family(0), nb_mothers_(0)
    {};
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_BRANCH_H
