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

  virtual void postReason() override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDesciption() override;

  virtual bool defaultAvtive() override {return true;}
private:
  void resolveChain(ObjectPropertyBranch_t* prop, std::vector<ObjectPropertyBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on);
  void resolveLink(ObjectPropertyBranch_t* chain_property, ChainTree* tree, size_t index);
  bool relationExists(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* chain_prop, IndividualBranch_t* chain_indiv);
  void addInduced(IndividualBranch_t* indiv, size_t index, IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);
};

class ChainNode_t
{
public:
  ChainNode_t() : on_(nullptr),
                  from_(nullptr),
                  prop_(nullptr),
                  prev(nullptr),
                  pose(0)
  {}

  ~ChainNode_t()
  {
    for(auto next : nexts)
      if(next != nullptr)
        delete next;
  }
  
  std::string toString()
  {
    return from_->value() + "|" + prop_->value() + "|" + on_->value();
  }

  void printTree(size_t level = 0)
  {
    for(size_t i = 0; i < level; i++)
      std::cout << " ";
    std::cout << "- " << toString() << std::endl;
    for(auto next : nexts)
      next->printTree(level+1);
  }

  IndividualBranch_t* on_;
  IndividualBranch_t* from_;
  ObjectPropertyBranch_t* prop_;
  ChainNode_t* prev;
  std::vector<ChainNode_t*> nexts;
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

  std::vector<ChainNode_t*> getNodes(size_t index)
  {
    if(begin != nullptr)
      return getNodes(begin, index, 0);
    else
      return {};
  }

  std::vector<ChainNode_t*> getChainTo(IndividualBranch_t* indiv)
  {
    if(begin != nullptr)
      return getChainTo(begin, indiv);
    else
      return {};
  }

  size_t size() { return size_; }

  void push(ChainNode_t* current, ChainNode_t* next)
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

  void print()
  {
    if(begin != nullptr)
      begin->printTree();
  }

  ChainNode_t* begin;

private:
  size_t size_;

  std::vector<IndividualBranch_t*> get(ChainNode_t* node, size_t index, size_t current)
  {
    std::vector<IndividualBranch_t*> res;
    if(current == index)
      res.push_back(node->on_);
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

  std::vector<ChainNode_t*> getNodes(ChainNode_t* node, size_t index, size_t current)
  {
    std::vector<ChainNode_t*> res;
    if(current == index)
      res.push_back(node);
    else
    {
      std::vector<ChainNode_t*> tmp;
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

  std::vector<ChainNode_t*> getChainTo(ChainNode_t* node, IndividualBranch_t* indiv)
  {
    std::vector<ChainNode_t*> res;
    if(node->on_ == indiv)
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

  bool purge(ChainNode_t* node, size_t size, size_t current)
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
      }
    }
    return tmp;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERCHAIN_H
