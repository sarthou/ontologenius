#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <vector>
#include <map>

#include "ontoloGenius/core/ontoGraphs/BranchContainer/BranchContainerMap.h"
#include "ontoloGenius/core/ontoGraphs/BranchContainer/BranchContainerDyn.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ValuedNode.h"

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

  BranchContainerMap<B> container_;

  std::string language_;

};

#endif
