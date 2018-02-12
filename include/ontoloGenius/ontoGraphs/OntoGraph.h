#ifndef ONTOGRAPH_H
#define ONTOGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdint.h>

#include "ontoloGenius/ontoGraphs/BranchContainerMap.h"
#include "ontoloGenius/ontoGraphs/BranchContainerDyn.h"

/*
This file use CRTP (curiously recurring template pattern)
be really carreful of how you use it
*/

template <typename T>
class Branch_t
{
public:
  std::string value_;
  std::vector<T*> childs_;
  std::vector<T*> mothers_;
  uint8_t family;
  uint8_t nb_mothers_;
  std::map<std::string, std::string> dictionary_;

  Branch_t(std::string value) : family(0), nb_mothers_(0)
    {value_ = value; };
};

template <typename B>
class OntoGraph
{
  static_assert(std::is_base_of<Branch_t<B>,B>::value, "B must be derived from Branch_t<B>");
public:
  OntoGraph() {}
  ~OntoGraph();

  void close();

  std::set<std::string> getDown(std::string value);
  std::set<std::string> getUp(std::string value);

  std::vector<B*> get()
  {
    std::vector<B*> out;
    out.insert( out.end(), branchs_.begin(), branchs_.end() );
    out.insert( out.end(), roots_.begin(), roots_.end() );
    return out;
  }

protected:
  BranchContainerDyn<B> container_;
  std::vector<B*> branchs_;
  std::vector<B*> roots_;

  std::vector<B*> tmp_mothers_;

  int depth_;

  void link();
  void add_family(B* branch, uint8_t family);

  std::set<std::string> getDown(B* branch, std::string value);
  std::set<std::string> getUp(B* branch, std::string value);
};

template <typename B>
OntoGraph<B>::~OntoGraph()
{
  for(unsigned int i = 0; i < branchs_.size(); i++)
    delete branchs_[i];

  for(unsigned int i = 0; i < roots_.size(); i++)
    delete roots_[i];

  branchs_.clear();
  roots_.clear();
}

template <typename B>
void OntoGraph<B>::close()
{
  for(unsigned int i = 0; i < tmp_mothers_.size(); i++)
    roots_.push_back(tmp_mothers_[i]);

  tmp_mothers_.clear();

  link();

  container_.load(roots_, branchs_);
}

template <typename B>
std::set<std::string> OntoGraph<B>::getDown(std::string value)
{
  std::set<std::string> res;

  B* branch = container_.find(value);
  std::set<std::string> tmp = getDown(branch, value);
  if(tmp.size())
    res.insert(tmp.begin(), tmp.end());

  return res;
}

template <typename B>
std::set<std::string> OntoGraph<B>::getUp(std::string value)
{
  std::set<std::string> res;

  B* branch = container_.find(value);
  std::set<std::string> tmp = getUp(branch, value);
  if(tmp.size())
    res.insert(tmp.begin(), tmp.end());

  return res;
}

template <typename B>
void OntoGraph<B>::link()
{
  depth_ = 0;

  uint8_t nb_root_family = roots_.size();
  for(uint8_t root_i = 0; root_i < roots_.size(); root_i++)
  {
    roots_[root_i]->family = 256/(nb_root_family+1) * root_i;
    for(unsigned int i = 0; i < roots_[root_i]->childs_.size(); i++)
      add_family(roots_[root_i]->childs_[i], roots_[root_i]->family);
  }
}

template <typename B>
void OntoGraph<B>::add_family(B* branch, uint8_t family)
{
  branch->family += family/branch->nb_mothers_;
  for(unsigned int i = 0; i < branch->childs_.size(); i++)
  {
    depth_++;
    if(depth_ < 20)
      add_family(branch->childs_[i], family/branch->nb_mothers_);
    depth_--;
  }
}

template <typename B>
std::set<std::string> OntoGraph<B>::getDown(B* branch, std::string value)
{
  std::set<std::string> res;
  res.insert(branch->value_);
  for(unsigned int i = 0; i < branch->childs_.size(); i++)
  {
    std::set<std::string> tmp = getDown(branch->childs_[i], value);

    if(tmp.size())
      res.insert(tmp.begin(), tmp.end());
  }

  return res;
}

template <typename B>
std::set<std::string> OntoGraph<B>::getUp(B* branch, std::string value)
{
  std::set<std::string> res;
  res.insert(branch->value_);
  for(unsigned int i = 0; i < branch->mothers_.size(); i++)
  {
    std::set<std::string> tmp = getUp(branch->mothers_[i], value);

    if(tmp.size())
      res.insert(tmp.begin(), tmp.end());
  }

  return res;
}

#endif
