#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <vector>
#include <map>

#include "ontoloGenius/core/ontoGraphs/BranchContainer/BranchContainerMap.h"
#include "ontoloGenius/core/ontoGraphs/BranchContainer/BranchContainerDyn.h"

class UpdatableNode
{
public:
  unsigned int nb_updates_;
  bool updated_;
  std::map<std::string, bool> flags_;
  UpdatableNode() {updated_ = true; nb_updates_ = 0; }
};

class ValuedNode : public UpdatableNode
{
public:
  std::string value_;
  std::map<std::string, std::vector<std::string>> dictionary_;

  ValuedNode(std::string value) {value_ = value; }
};

template <typename B>
class Graph
{
  static_assert(std::is_base_of<ValuedNode,B>::value, "B must be derived from ValuedNode");
public:
  Graph() {language_ = "en"; }
  virtual ~Graph() {}

  void setLanguage(std::string language) {language_ = language; }

  virtual void close() = 0;

  virtual std::vector<B*> get() = 0;

  BranchContainerDyn<B> container_;

  std::string language_;

};

#endif
