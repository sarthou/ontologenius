#ifndef BRANCHCONTAINERMAP_H
#define BRANCHCONTAINERMAP_H

#include <map>

#include "ontoloGenius/ontoGraphs/BranchContainerBase.h"

template <typename B>
class BranchContainerMap : public BranchContainerBase<B>
{
public:
  BranchContainerMap() {}
  virtual ~BranchContainerMap() {}

  virtual B* find(std::string word);
  virtual void load(std::vector<B*> roots, std::vector<B*> branchs);
private:
  std::map<std::string, B*> nodes_;
};

template <typename B>
B* BranchContainerMap<B>::find(std::string word)
{
  typename std::map<std::string, B*>::iterator it = nodes_.find(word);
  if(it == nodes_.end())
    return nullptr;
  else
    return it->second;
}

template <typename B>
void BranchContainerMap<B>::load(std::vector<B*> roots, std::vector<B*> branchs)
{
  for(size_t i = 0; i < roots.size(); i++)
    nodes_[roots[i]->value_] = roots[i];

  for(size_t i = 0; i < branchs.size(); i++)
    nodes_[branchs[i]->value_] = branchs[i];
}

#endif
