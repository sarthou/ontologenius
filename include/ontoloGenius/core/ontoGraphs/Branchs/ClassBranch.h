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
class ClassSteady_t : public BranchSteady_t<ClassBranch_t>, public ClassBranchData_t<ClassBranch_t>
{
public:
  ClassSteady_t() {}
};

class ClassBranch_t : public Branch_t<ClassBranch_t>, public ClassBranchData_t<ClassBranch_t>
{
public:
  ClassSteady_t steady_;

  ClassBranch_t(std::string value) : Branch_t(value) {};

  void setFullSteady();
  void setSteady_disjoint(ClassBranch_t* disjoint);
  void setSteady_child(ClassBranch_t* child);
  void setSteady_mother(ClassBranch_t* mother);
  void setSteady_dictionary(std::string lang, std::string word);
  void setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
};

#endif
