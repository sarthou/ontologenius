#include "ontologenius/graphical/Drawers/DataPropertyDrawer.h"

#include <iostream>

namespace ontologenius {

DataPropertyDrawer::DataPropertyDrawer(DataPropertyGraph* graph)
{
  graph_ = graph;
  init();
}

void DataPropertyDrawer::putInLayers()
{
  if((graph_ != nullptr) && (graph_->roots_.size() != 0))
  {
    //init markers
    for(auto& branch : branchs_nodes)
      branch->marker = false;

    for(size_t i = 0; i < roots_nodes.size(); i++)
    {
      roots_nodes[i]->marker = false;
      roots_nodes[i]->pos = i;
    }

    int layer = 0;

    while(!testEnd())
    {
      layer_nodes.emplace_back();
      putLayer(layer);
      layer++;
    }

    if(layer_nodes.size() > 0)
      layer_nodes.pop_back();
  }
}

int DataPropertyDrawer::createNode(DataPropertyBranch_t* branch, node_t* mother)
{
  int family = branch->family;
  if(!exist(branch->value()))
  {
    auto node = new node_t(branch->value());
    branchs_nodes.push_back(node);
    node->prev.push_back(mother);
    node->family = branch->family;
    for(auto& child : branch->childs_)
      family += createNode(child.elem, node);

    family = family / (branch->childs_.size() + 1);
  }
  else
  {
    for(auto& branch_node : branchs_nodes)
      if(branch_node->value == branch->value())
        branch_node->prev.push_back(mother);
  }
  return family;
}

void DataPropertyDrawer::init()
{
  std::vector<node_t*> single;
  std::vector<node_t*> couple;
  if(graph_ != nullptr)
  {
    for(auto& it : graph_->roots_)
    {
      auto node = new node_t(it.second->value(), 0);
      //roots_nodes.push_back(node);
      node->family = it.second->family;
      int family = it.second->family;

      for(auto& child : it.second->childs_)
        family += createNode(child.elem, node);

      family = family / (it.second->childs_.size() + 1);
      if(family == node->family)
        single.push_back(node);
      else
        couple.push_back(node);
    }

    size_t middle = single.size()/2;
    for(size_t i = 0; i < middle; i++)
      roots_nodes.push_back(single[i]);

    for(size_t i = 0; i < couple.size(); i++)
      roots_nodes.push_back(couple[i]);

    for(size_t i = middle; i < single.size(); i++)
      roots_nodes.push_back(single[i]);
  }
}

} // namespace ontologenius
