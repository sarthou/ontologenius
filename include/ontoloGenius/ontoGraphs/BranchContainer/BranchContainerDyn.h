#ifndef BRANCHCONTAINERDYN_H
#define BRANCHCONTAINERDYN_H

#include <map>
#include <iostream>
#include <math.h>

#include "ontoloGenius/ontoGraphs/BranchContainer/BranchContainerBase.h"

template <typename T>
struct BramchNode_t
{
  BramchNode_t* next;
  std::string id;
  T* branch;

  BramchNode_t() {next = nullptr; id = ""; branch = nullptr; }
  ~BramchNode_t() { delete next; }
};

template <typename B>
class BranchContainerDyn : public BranchContainerBase<B>
{
public:
  BranchContainerDyn()
  {
    nodes_ = new BramchNode_t<B>;
    nodes_end_ = nodes_;
    nb_elem_ = 0;
  }

  virtual ~BranchContainerDyn() {delete nodes_; }

  virtual B* find(std::string word);
  virtual B* find(bool (*comp)(B*, std::string, std::string), std::string word, std::string lang) {};
  virtual void load(std::vector<B*>& vect);
private:
  BramchNode_t<B>* nodes_;
  BramchNode_t<B>* nodes_end_;
  size_t buffer_size_;
  size_t nb_elem_;

  void insertEnd(std::string id, B* branch);
  void reconf(BramchNode_t<B>* node);
};

template <typename B>
B* BranchContainerDyn<B>::find(std::string word)
{
  B* tmp = nullptr;
  size_t i = 0;
  for(BramchNode_t<B>* node = nodes_; node->next != nullptr; node = node->next)
  {
    if(node->next->id == word)
    {
      tmp = node->next->branch;
      if(i > 5)
        reconf(node);
      break;
    }
    i++;
  }

  return tmp;
}

template <typename B>
void BranchContainerDyn<B>::load(std::vector<B*>& vect)
{
  for(size_t i = 0; i < vect.size(); i++)
    insertEnd(vect[i]->value_, vect[i]);

  nb_elem_ += vect.size();
  buffer_size_ = log2(nb_elem_);
}

template <typename B>
void BranchContainerDyn<B>::insertEnd(std::string id, B* branch)
{
  BramchNode_t<B>* tmp = new BramchNode_t<B>;
  tmp->id = id;
  tmp->branch = branch;
  nodes_end_->next = tmp;
  nodes_end_ = tmp;
}

template <typename B>
void BranchContainerDyn<B>::reconf(BramchNode_t<B>* node)
{
 BramchNode_t<B>* tmp = node->next;
 node->next = tmp->next;
 tmp->next = nodes_->next;
 nodes_->next = tmp;
}

#endif
