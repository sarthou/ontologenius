#ifndef ONTOLOGENIUS_BRANCHCONTAINERMAP_H
#define ONTOLOGENIUS_BRANCHCONTAINERMAP_H

#include <map>
#include <unordered_map>

#include "ontologenius/core/ontoGraphs/BranchContainer/BranchContainerBase.h"

namespace ontologenius {

template <typename B>
class BranchContainerMap : public BranchContainerBase<B>
{
public:
  BranchContainerMap() {}
  BranchContainerMap(const BranchContainerMap& base);
  virtual ~BranchContainerMap() {} //B* is destructed by ontograph

  virtual B* find(const std::string& word);
  virtual std::vector<B*> find(bool (*comp)(B*, const std::string&, const std::string&, bool), const std::string& word, const std::string& lang, bool use_default);
  virtual void load(std::vector<B*>& vect);
  virtual void insert(B* branch);
  virtual void erase(B* branch);
private:
  std::unordered_map<std::string, B*> nodes_;
};

template <typename B>
BranchContainerMap<B>::BranchContainerMap(const BranchContainerMap& base)
{
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
  typename std::unordered_map<std::string, B*>::iterator it = nodes_.find(word);
  if(it == nodes_.end())
    return nullptr;
  else
    return it->second;
}

template <typename B>
std::vector<B*> BranchContainerMap<B>::find(bool (*comp)(B*, const std::string&, const std::string&, bool), const std::string& word, const std::string& lang, bool use_default)
{
  std::vector<B*> res;

  for(auto& it : nodes_)
  {
    try {
      if(comp(it.second, word, lang, use_default))
        res.push_back(it.second);
    } catch(...) {
      return res;
    }
  }

  return res;
}

template <typename B>
void BranchContainerMap<B>::load(std::vector<B*>& vect)
{
  for(size_t i = 0; i < vect.size(); i++)
    nodes_[vect[i]->value()] = vect[i];
}

template <typename B>
void BranchContainerMap<B>::insert(B* branch)
{
  nodes_[branch->value()] = branch;
}

template <typename B>
void BranchContainerMap<B>::erase(B* branch)
{
  nodes_.erase(branch->value());
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_BRANCHCONTAINERMAP_H
