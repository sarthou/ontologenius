#ifndef BRANCHCONTAINERMAP_H
#define BRANCHCONTAINERMAP_H

#include <map>

#include "ontoloGenius/core/ontoGraphs/BranchContainer/BranchContainerBase.h"

template <typename B>
class BranchContainerMap : public BranchContainerBase<B>
{
public:
  BranchContainerMap() {}
  virtual ~BranchContainerMap() {}

  virtual B* find(std::string word);
  virtual std::vector<B*> find(bool (*comp)(B*, std::string, std::string), std::string word, std::string lang);
  virtual void load(std::vector<B*>& vect);
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
std::vector<B*> BranchContainerMap<B>::find(bool (*comp)(B*, std::string, std::string), std::string word, std::string lang)
{
  std::vector<B*> res;

  typename std::map<std::string, B*>::iterator it;
  for(it = nodes_.begin(); it != nodes_.end(); ++it)
    if(comp(it->second, word, lang))
      res.push_back(it->second);
  return res;
}

template <typename B>
void BranchContainerMap<B>::load(std::vector<B*>& vect)
{
  for(size_t i = 0; i < vect.size(); i++)
    nodes_[vect[i]->value_] = vect[i];
}

#endif
