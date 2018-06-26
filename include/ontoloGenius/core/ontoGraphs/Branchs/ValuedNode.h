#ifndef VALUEDNODE_H
#define VALUEDNODE_H

#include <string>
#include <vector>
#include <map>

class UpdatableNode
{
public:
  unsigned int nb_updates_;
  bool updated_;
  std::map<std::string, bool> flags_;
  UpdatableNode() {updated_ = true; nb_updates_ = 0; }
};

class ValuedNodeData
{
public:
  std::map<std::string, std::vector<std::string>> dictionary_;
};

class ValuedNode : public UpdatableNode, public ValuedNodeData
{
public:
  std::string value_;

  ValuedNode(std::string value) {value_ = value; }
};

#endif
