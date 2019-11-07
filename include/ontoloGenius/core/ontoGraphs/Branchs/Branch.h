#ifndef ONTOLOGENIUS_BRANCH_H
#define ONTOLOGENIUS_BRANCH_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/ValuedNode.h"

/*
This file use CRTP (curiously recurring template pattern)
be really carreful of how you use it
*/

namespace ontologenius {

template <typename B>
class BranchData_t
{
public:
  std::vector<Single_t<B*>> childs_;
  std::vector<Single_t<B*>> mothers_;
};

template <typename T>
class Branch_t : public BranchData_t<T>, public ValuedNode
{
public:
  uint8_t family;
  uint8_t nb_mothers_;

  Branch_t(std::string value) : ValuedNode(value), family(0), nb_mothers_(0)
    {};
};

template <typename S>
class BranchSteady_t : public ValuedNodeData, public BranchData_t<S>
{
public:
  BranchSteady_t() {}
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_BRANCH_H
