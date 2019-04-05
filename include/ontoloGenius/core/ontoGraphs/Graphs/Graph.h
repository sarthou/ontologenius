#ifndef ONTOLOGENIUS_GRAPH_H
#define ONTOLOGENIUS_GRAPH_H

#include <string>
#include <vector>
#include <map>
#include <mutex>  // For std::unique_lock
#include <shared_mutex>

#include "ontoloGenius/core/ontoGraphs/BranchContainer/BranchContainerMap.h"
#include "ontoloGenius/core/ontoGraphs/BranchContainer/BranchContainerDyn.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

template <typename B>
class Graph
{
  static_assert(std::is_base_of<ValuedNode,B>::value, "B must be derived from ValuedNode");
public:
  Graph() { language_ = "en"; }
  virtual ~Graph() {}

  void setLanguage(std::string language) {language_ = language; }
  std::string getLanguage() {return language_; }

  virtual void close() = 0;

  virtual std::vector<B*> get() = 0;
  virtual B* findBranch(const std::string& name);
  virtual B* findBranchUnsafe(const std::string& name);
  virtual B* create(const std::string& name);

  BranchContainerMap<B> container_;

  std::string language_;

  mutable std::shared_timed_mutex mutex_;
  //use std::lock_guard<std::shared_timed_mutex> lock(Graph<B>::mutex_); to WRITE A DATA
  //use std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_); to READ A DATA
};

template <typename B>
B* Graph<B>::findBranch(const std::string& name)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return container_.find(name);
}

template <typename B>
B* Graph<B>::findBranchUnsafe(const std::string& name)
{
  return container_.find(name);
}

template <typename B>
B* Graph<B>::create(const std::string& name)
{
  B* indiv = nullptr;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    indiv = container_.find(name);
  }

  if(indiv == nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    indiv = new B(name);
    container_.insert(indiv);
  }
  return indiv;
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_GRAPH_H
