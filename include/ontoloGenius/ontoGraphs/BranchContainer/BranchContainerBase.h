#ifndef BRANCHCONTAINERBASE_H
#define BRANCHCONTAINERBASE_H

#include <string>
#include <vector>

class ValuedNode;

template <typename B>
class BranchContainerBase
{
  static_assert(std::is_base_of<ValuedNode,B>::value, "B must be derived from ValuedNode");
  //static_assert(std::is_pointer<B>::value, "B must be a pointer");
public:
  BranchContainerBase() {}
  virtual ~BranchContainerBase() {}

  virtual B* find(std::string word) = 0;
  virtual void load(std::vector<B*>& vect) = 0;
private:
};

#endif
