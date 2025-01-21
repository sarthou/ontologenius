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

  template<typename T>
  class Branch : public ValuedNode
  {
  public:
    std::vector<SingleElement<T*>> childs_;
    RelationsWithInductions<SingleElement<T*>> mothers_;
    std::vector<SingleElement<T*>> disjoints_;

    explicit Branch(const std::string& value, bool hidden = false) : ValuedNode(value, hidden) {}

    void setUpdated(bool value)
    {
      if(value == false)
        mothers_.resetUpdated();
      ValuedNode::setUpdated(value);
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_BRANCH_H
