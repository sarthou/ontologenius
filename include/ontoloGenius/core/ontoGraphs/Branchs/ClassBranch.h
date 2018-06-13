#ifndef CLASSBRANCH_H
#define CLASSBRANCH_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/Branch.h"

template <typename T>
class ClassBranchData_t
{
public:
  std::vector<T*> disjoints_;
};

class ClassBranch_t;
class ClassSteady_t : public BranchData_t<ClassBranch_t>, public ClassBranchData_t<ClassBranch_t>
{
public:
  ClassSteady_t() {}
};

class ClassBranch_t : public Branch_t<ClassBranch_t>, public ClassBranchData_t<ClassBranch_t>
{
public:
  ClassSteady_t steady_;

  ClassBranch_t(std::string value) : Branch_t(value) {};
};

#endif
