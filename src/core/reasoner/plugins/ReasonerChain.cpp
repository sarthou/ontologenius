#include "ontoloGenius/core/reasoner/plugins/ReasonerChain.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerChain::preReason()
{

}

void ReasonerChain::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();
  for(size_t indiv_i = 0; indiv_i < indiv_size; indiv_i++)
    if(indiv[indiv_i]->updated_ == true)
    {
      for(size_t prop_i = 0; prop_i < indiv[indiv_i]->object_properties_name_.size(); prop_i++)
      {
        std::unordered_set<ObjectPropertyBranch_t*> props = ontology_->object_property_graph_.getUpPtrSafe(indiv[indiv_i]->object_properties_name_[prop_i]);
        for(ObjectPropertyBranch_t* it_prop : props)
          for(size_t chain_i = 0; chain_i < it_prop->chains_.size(); chain_i++)
            resolveChain(it_prop, it_prop->chains_[chain_i], indiv[indiv_i]->object_properties_on_[prop_i], indiv[indiv_i]);
      }
    }
}

void ReasonerChain::resolveChain(ObjectPropertyBranch_t* prop, std::vector<ObjectPropertyBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on)
{
  chainNode_t* indivs_node = new chainNode_t;
  indivs_node->ons_.push_back(indiv);
  indivs_node->froms_.push_back(on);
  indivs_node->props_.push_back(prop);

  ChainTree tree;
  tree.push(nullptr, indivs_node);

  size_t chain_size = chain.size() - 1;
  for(size_t link_i = 0; link_i < chain_size; link_i++)
  {
    resolveLink(chain[link_i], &tree, link_i);
  }

  tree.purge(chain_size);

  std::vector<IndividualBranch_t*> indivs = tree.get(chain_size);
  size_t indivs_size = indivs.size();
  if((chain.size() != 0) && (indivs_size != 0))
    for(size_t i = 0; i < indivs_size; i++)
      if(!porpertyExist(on, chain[chain_size], indivs[i]))
      {
        on->object_properties_name_.push_back(chain[chain_size]);
        on->object_properties_on_.push_back(indivs[i]);
        on->object_properties_deduced_.push_back(false);
        on->object_properties_has_induced_.push_back(Triplet());
        on->updated_ = true;
        for(auto branch : on->object_properties_on_)
          branch->updated_ = true;
        on->nb_updates_++;
        nb_update_++;

        for(size_t link_i = 0; link_i < chain.size(); link_i++)
        {
          std::vector<chainNode_t*> nodes = tree.getNodes(link_i);
          for(size_t node_i = 0; node_i < nodes.size(); node_i++)
            for(size_t node_it = 0; node_it < nodes[node_i]->ons_.size(); node_it++)
            {
              for(size_t prop_i = 0; prop_i < nodes[node_i]->froms_[node_it]->object_properties_name_.size(); prop_i++)
                if(nodes[node_i]->froms_[node_it]->object_properties_name_[prop_i] == nodes[node_i]->props_[node_it])
                  if(nodes[node_i]->froms_[node_it]->object_properties_on_[prop_i] == nodes[node_i]->ons_[node_it])
                    addInduced(nodes[node_i]->froms_[node_it], prop_i, on, chain[chain_size], indivs[i]);
            }
        }
      }
}

void ReasonerChain::resolveLink(ObjectPropertyBranch_t* chain_property, ChainTree* tree, size_t index)
{
  std::vector<IndividualBranch_t*> tmp_on;
  std::vector<IndividualBranch_t*> tmp_from;
  std::vector<ObjectPropertyBranch_t*> tmp_prop;
  std::unordered_set<std::string> chain_props = ontology_->object_property_graph_.getDown(chain_property->value());

  std::vector<chainNode_t*> nodes = tree->getNodes(index);
  for(auto node : nodes)
  {
    for(auto indiv : node->ons_)
    {
      tmp_on.clear();
      tmp_from.clear();
      for(size_t prop_i = 0; prop_i < indiv->object_properties_name_.size(); prop_i++)
      {
        if(chain_props.find(indiv->object_properties_name_[prop_i]->value()) != chain_props.end())
        {
          tmp_on.push_back(indiv->object_properties_on_[prop_i]);
          tmp_from.push_back(indiv);
          tmp_prop.push_back(indiv->object_properties_name_[prop_i]);
        }
      }
      if(tmp_on.size() != 0)
      {
        chainNode_t* indivs = new chainNode_t;
        indivs->ons_ = tmp_on;
        indivs->froms_ = tmp_from;
        indivs->props_ = tmp_prop;
        tree->push(node, indivs);
      }
    }
  }
}

bool ReasonerChain::porpertyExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* chain_prop, IndividualBranch_t* chain_indiv)
{
  size_t properties_size = indiv_on->object_properties_name_.size();
  for(size_t i = 0; i < properties_size; i++)
  {
    if(indiv_on->object_properties_name_[i]->get() == chain_prop->get())
      if(indiv_on->object_properties_on_[i]->get() == chain_indiv->get())
        return true;
  }
  return false;
}

void ReasonerChain::addInduced(IndividualBranch_t* indiv, size_t index, IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  if(indiv->object_properties_has_induced_[index].exist(indiv_from, property, indiv_on) == false)
    indiv->object_properties_has_induced_[index].push(indiv_from, property, indiv_on);
}

std::string ReasonerChain::getName()
{
  return "reasoner chain";
}

std::string ReasonerChain::getDesciption()
{
  return "This reasoner resolve the properties chains axioms.";
}

PLUGINLIB_EXPORT_CLASS(ReasonerChain, ReasonerInterface)

} // namespace ontologenius
