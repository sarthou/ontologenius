#ifndef ONTOLOGENIUS_REASONERCHAIN_H
#define ONTOLOGENIUS_REASONERCHAIN_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ChainTree;
class ReasonerChain : public ReasonerInterface
{
public:
  ReasonerChain() {}
  ~ReasonerChain() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:
  void resolveChain(ObjectPropertyBranch_t* prop, std::vector<ObjectPropertyBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on);
  void resolveLink(ObjectPropertyBranch_t* chain_property, ChainTree* tree, size_t index);
  bool relationExists(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* chain_prop, IndividualBranch_t* chain_indiv);
  void addInduced(IndividualBranch_t* indiv, size_t index, IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);
};

class chainNode_t
{
public:
  ~chainNode_t()
  {
    for(auto next : nexts)
      if(next != nullptr)
        delete next;
  }
  chainNode_t()
  {
    prev = nullptr;
    pose = 0;
  }

  std::string toString()
  {
    std::string res;
    for(size_t i = 0; i < ons_.size(); i++)
      res += froms_[i]->value() + "|" + props_[i]->value() + "|" + ons_[i]->value() + (i != 0 ? ";" : "");
    return res;
  }

  std::vector<IndividualBranch_t*> ons_;
  std::vector<IndividualBranch_t*> froms_;
  std::vector<ObjectPropertyBranch_t*> props_;
  chainNode_t* prev;
  std::vector<chainNode_t*> nexts;
  size_t pose;
};

class ChainTree
{
public:
  ChainTree()
  {
    size_ = 0;
    begin = nullptr;
  }

  ~ChainTree()
  {
    if(begin != nullptr)
      delete begin;
  }

  std::vector<IndividualBranch_t*> get(size_t index)
  {
    if(begin != nullptr)
      return get(begin, index, 0);
    else
      return {};
  }

  std::vector<chainNode_t*> getNodes(size_t index)
  {
    if(begin != nullptr)
      return getNodes(begin, index, 0);
    else
      return {};
  }

  std::vector<chainNode_t*> getChainTo(IndividualBranch_t* indiv)
  {
    if(begin != nullptr)
      return getChainTo(begin, indiv);
    else
      return {};
  }

  size_t size() { return size_; }

  void push(chainNode_t* current, chainNode_t* next)
  {
    if(current != nullptr)
    {
      next->pose = current->pose + 1;
      if(next->pose > size_)
        size_ = next->pose;
      next->prev = current;
      current->nexts.push_back(next);
    }
    else
      begin = next;
  }

  void purge(size_t size)
  {
    if(begin != nullptr)
      purge(begin, size, 0);
  }

  chainNode_t* begin;

private:
  size_t size_;

  std::vector<IndividualBranch_t*> get(chainNode_t* node, size_t index, size_t current)
  {
    std::vector<IndividualBranch_t*> res;
    if(current == index)
      res = node->ons_;
    else
    {
      std::vector<IndividualBranch_t*> tmp;
      for(auto next : node->nexts)
      {
        if(next != nullptr)
        {
          tmp = get(next, index, current + 1);
          res.insert(res.end(), tmp.begin(), tmp.end());
        }
      }
    }
    return res;
  }

  std::vector<chainNode_t*> getNodes(chainNode_t* node, size_t index, size_t current)
  {
    std::vector<chainNode_t*> res;
    if(current == index)
      res.push_back(node);
    else
    {
      std::vector<chainNode_t*> tmp;
      for(auto next : node->nexts)
      {
        if(next != nullptr)
        {
          tmp = getNodes(next, index, current + 1);
          res.insert(res.end(), tmp.begin(), tmp.end());
        }
      }
    }
    return res;
  }

  std::vector<chainNode_t*> getChainTo(chainNode_t* node, IndividualBranch_t* indiv)
  {
    std::vector<chainNode_t*> res;
    if(std::find(node->ons_.begin(), node->ons_.end(), indiv) != node->ons_.end())
      res.push_back(node);
    else
    {
      for(auto next : node->nexts)
      {
        if(next != nullptr)
        {
          res = getChainTo(next, indiv);
          if(res.size())
          {
            res.push_back(node);
            return res;
          }
        }
      }
    }
    return res;
  }

  bool purge(chainNode_t* node, size_t size, size_t current)
  {
    bool tmp = true;
    if(node != nullptr)
    {
      if(current < size)
      {
        for(auto next : node->nexts)
          tmp = tmp && purge(next, size, current + 1);
      }
      else
        tmp = false;

      if(tmp == true)
      {
        if(node->prev != nullptr)
        {
          for(size_t i = 0; i < node->prev->nexts.size(); )
            if(node->prev->nexts[i] == node)
              node->prev->nexts.erase(node->prev->nexts.begin() + i);
        }

        if(node == begin)
          begin = nullptr;
        delete node;
        node = nullptr;
      }
    }
    return tmp;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERCHAIN_H
