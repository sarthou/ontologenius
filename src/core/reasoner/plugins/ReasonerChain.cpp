#include "ontologenius/core/reasoner/plugins/ReasonerChain.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerChain::preReason()
{

}

void ReasonerChain::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.get();
  for(auto indiv : indivs)
    if(indiv->updated_ == true)
    {
      for(auto& relation : indiv->object_relations_)
      {
        std::unordered_set<ObjectPropertyBranch_t*> props = ontology_->object_property_graph_.getUpPtrSafe(relation.first);
        for(ObjectPropertyBranch_t* it_prop : props)
          for(auto& chain : it_prop->chains_)
            resolveChain(it_prop, chain, relation.second, indiv);
      }
    }
}

void ReasonerChain::resolveChain(ObjectPropertyBranch_t* prop, std::vector<ObjectPropertyBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on)
{
  auto indivs_node = new chainNode_t;
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
        on->object_relations_.emplace_back(chain[chain_size], indivs[i], 1.0, true);
        on->object_properties_has_induced_.emplace_back();
        on->updated_ = true;
        for(auto relation : on->object_relations_)
          relation.second->updated_ = true;
        std::string explanation_reference;
        auto link_chain = tree.getChainTo(indivs[i]);
        for(auto& lc : link_chain)
        {
          if(explanation_reference != "") explanation_reference += ";";
          explanation_reference += lc->toString();
        }
        explanations_.emplace_back("[ADD]" + on->value() + "|" + chain[chain_size]->value() + "|" + indivs[i]->value(),
                                   "[ADD]" + explanation_reference);

        on->nb_updates_++;
        nb_update_++;

        for(size_t link_i = 0; link_i < chain.size(); link_i++)
        {
          std::vector<chainNode_t*> nodes = tree.getNodes(link_i);
          for(auto& node : nodes)
            for(size_t node_it = 0; node_it < node->ons_.size(); node_it++)
            {
              for(size_t prop_i = 0; prop_i < node->froms_[node_it]->object_relations_.size(); prop_i++)
              {
                std::unordered_set<ObjectPropertyBranch_t*> prop_down;
                ontology_->object_property_graph_.getDownPtr(node->props_[node_it], prop_down);
                for(auto& down : prop_down)
                {
                  if(node->froms_[node_it]->object_relations_[prop_i].first == down)
                    if(node->froms_[node_it]->object_relations_[prop_i].second == node->ons_[node_it])
                    {
                      addInduced(node->froms_[node_it], prop_i, on, chain[chain_size], indivs[i]);
                      break;
                    }
                }
              }
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
      for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
      {
        if(chain_props.find(relation.first->value()) != chain_props.end())
        {
          tmp_on.push_back(relation.second);
          tmp_from.push_back(indiv);
          tmp_prop.push_back(relation.first);
        }
      }
      if(tmp_on.size() != 0)
      {
        auto indivs = new chainNode_t;
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
  size_t properties_size = indiv_on->object_relations_.size();
  for(size_t i = 0; i < properties_size; i++)
  {
    if(indiv_on->object_relations_[i].first->get() == chain_prop->get())
      if(indiv_on->object_relations_[i].second->get() == chain_indiv->get())
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
