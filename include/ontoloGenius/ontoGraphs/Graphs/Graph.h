#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <vector>

#include "ontoloGenius/ontoGraphs/BranchContainer/BranchContainerMap.h"
#include "ontoloGenius/ontoGraphs/BranchContainer/BranchContainerDyn.h"

class ValuedNode
{
public:
  std::string value_;

  ValuedNode(std::string value) {value_ = value; }
};

template <typename B>
class Graph
{
  static_assert(std::is_base_of<ValuedNode,B>::value, "B must be derived from ValuedNode");
public:
  Graph() {language_ = "en"; }
  ~Graph() {}

  void setLanguage(std::string language) {language_ = language; }

  virtual void close() = 0;

  virtual std::vector<B*> get() = 0;

  BranchContainerMap<B> container_;

  std::string language_;

};

#endif
