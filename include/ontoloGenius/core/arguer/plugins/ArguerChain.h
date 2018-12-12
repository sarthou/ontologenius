#ifndef ARGUERCHAIN_H
#define ARGUERCHAIN_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

class ChainTree;
class ArguerChain : public ArguerInterface
{
public:
  ArguerChain() {}
  ~ArguerChain() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:
  void resolveChain(ObjectPropertyBranch_t* prop, std::vector<ObjectPropertyBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on);
  void resolveLink(ObjectPropertyBranch_t* chain_property, ChainTree* tree, size_t index);
  bool porpertyExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* chain_prop, IndividualBranch_t* chain_indiv);
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
  std::vector<IndividualBranch_t*> data;
  std::vector<IndividualBranch_t*> data_from;
  std::vector<ObjectPropertyBranch_t*> prop;
  chainNode_t* prev;
  std::vector<chainNode_t*> nexts;
  size_t pose;
};

class ChainTree
{
public:
  ChainTree() {size_ = 0; begin = nullptr;}
  std::vector<IndividualBranch_t*> get(size_t index)
  {
    std::vector<IndividualBranch_t*> res;
    if(begin != nullptr)
      res = get(begin, index, 0);
    return res;
  }
  std::vector<chainNode_t*> getNodes(size_t index)
  {
    std::vector<chainNode_t*> res;
    if(begin != nullptr)
      res = getNodes(begin, index, 0);
    return res;
  }
  size_t size() {return size_; }
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
  std::vector<IndividualBranch_t*> get(chainNode_t* node, size_t index, size_t current)
  {
    std::vector<IndividualBranch_t*> res;
    if(current == index)
      res = node->data;
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

        delete node;
      }
    }
    return tmp;
  }

  size_t size_;
};

#endif
