#include "ontoloGenius/ontoGraphs/Drawers/ClassDrawer.h"
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

ClassDrawer::ClassDrawer(ClassGraph* p_tree)
{
  m_tree = p_tree;
  init();
}

void ClassDrawer::put_in_layers()
{
  if((m_tree != nullptr) && (m_tree->roots_.size() != 0))
  {
    //init markers
    for(unsigned long int i = 0; i < branchs_nodes.size(); i++)
      branchs_nodes[i]->marker = false;

    for(unsigned long int i = 0; i < roots_nodes.size(); i++)
    {
      roots_nodes[i]->marker = false;
      roots_nodes[i]->pos = i;
    }

    int layer = 0;

    while(!test_end())
    {
      layer_nodes.push_back(std::vector<node_t*>());
      put_layer(layer);
      layer++;
    }
    layer_nodes.pop_back();
  }
}

int ClassDrawer::create_node(ClassBranch_t* branch, node_t* mother)
{
  int family = branch->family;
  if(!exist(branch->value_))
  {
    node_t* node = new node_t(branch->value_);
    branchs_nodes.push_back(node);
    node->prev.push_back(mother);
    node->family = branch->family;
    for(unsigned long int i = 0; i < branch->childs_.size(); i++)
      family += create_node(branch->childs_[i], node);

    family = family / (branch->childs_.size() + 1);
  }
  else
  {
    for(unsigned long int i = 0; i < branchs_nodes.size(); i++)
      if(branchs_nodes[i]->value == branch->value_)
        branchs_nodes[i]->prev.push_back(mother);
  }

}

void ClassDrawer::init()
{
  std::vector<node_t*> single;
  std::vector<node_t*> couple;
  if(m_tree != nullptr)
  {
    for(unsigned long int i = 0; i < m_tree->roots_.size(); i++)
    {
      node_t* node = new node_t(m_tree->roots_[i]->value_, 0);
      //roots_nodes.push_back(node);
      node->family = m_tree->roots_[i]->family;
      int family = m_tree->roots_[i]->family;

      for(unsigned long int branch = 0; branch < m_tree->roots_[i]->childs_.size(); branch++)
        family += create_node(m_tree->roots_[i]->childs_[branch], node);

      family = family / (m_tree->roots_[i]->childs_.size() + 1);
      if(family == node->family)
        single.push_back(node);
      else
        couple.push_back(node);
    }

    int middle = single.size()/2;
    for(unsigned long int i = 0; i < middle; i++)
      roots_nodes.push_back(single[i]);

    for(unsigned long int i = 0; i < couple.size(); i++)
      roots_nodes.push_back(couple[i]);

    for(unsigned long int i = middle; i < single.size(); i++)
      roots_nodes.push_back(single[i]);
  }
}
