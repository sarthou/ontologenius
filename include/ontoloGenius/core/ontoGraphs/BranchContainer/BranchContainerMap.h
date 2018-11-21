#ifndef BRANCHCONTAINERMAP_H
#define BRANCHCONTAINERMAP_H

#include <map>
#include <iostream> //TODO REMOVE

#include "ontoloGenius/core/ontoGraphs/BranchContainer/BranchContainerBase.h"

template <typename B>
class BranchContainerMap : public BranchContainerBase<B>
{
public:
  BranchContainerMap() {}
  BranchContainerMap(const BranchContainerMap& base);
  virtual ~BranchContainerMap() {} //B* is destructed by ontograph

  virtual B* find(const std::string& word);
  virtual std::vector<B*> find(bool (*comp)(B*, std::string, std::string), const std::string& word, const std::string& lang);
  virtual void load(std::vector<B*>& vect);
private:
  std::map<std::string, B*> nodes_;
};

template <typename B>
BranchContainerMap<B>::BranchContainerMap(const BranchContainerMap& base)
{
  std::cout << "BranchContainerMap have been copied" << std::endl; //TODO REMOVE
  for(auto& it : base.nodes_)
  {
    B* tmp = new B();
    *tmp = *(it.second);
    nodes_[it.first] = tmp;
  }
}

template <typename B>
B* BranchContainerMap<B>::find(const std::string& word)
{
  typename std::map<std::string, B*>::iterator it = nodes_.find(word);
  if(it == nodes_.end())
    return nullptr;
  else
    return it->second;
}

template <typename B>
std::vector<B*> BranchContainerMap<B>::find(bool (*comp)(B*, std::string, std::string), const std::string& word, const std::string& lang)
{
  std::vector<B*> res;

  for(auto& it : nodes_)
    if(comp(it.second, word, lang))
      res.push_back(it.second);
  return res;
}

template <typename B>
void BranchContainerMap<B>::load(std::vector<B*>& vect)
{
  for(size_t i = 0; i < vect.size(); i++)
    nodes_[vect[i]->value()] = vect[i];
}

#endif
