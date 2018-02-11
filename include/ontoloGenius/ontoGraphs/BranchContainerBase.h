#ifndef BRANCHCONTAINERBASE_H
#define BRANCHCONTAINERBASE_H

#include <string>
#include <vector>

template <typename T> class Branch_t;

template <typename B>
class BranchContainerBase
{
  static_assert(std::is_base_of<Branch_t<B>,B>::value, "B must be derived from Branch_t");
  //static_assert(std::is_pointer<B>::value, "B must be a pointer");
public:
  BranchContainerBase() {}
  virtual ~BranchContainerBase() {}

  virtual B* find(std::string word) = 0;
  virtual void load(std::vector<B*> roots, std::vector<B*> branchs) = 0;
private:
};

#endif
