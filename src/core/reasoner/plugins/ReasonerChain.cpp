#include "ontologenius/core/reasoner/plugins/ReasonerChain.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerChain::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.get();
  for(auto indiv : indivs)
    if((indiv->updated_ == true) || (indiv->flags_.find("chain") != indiv->flags_.end()))
    {
      bool has_active_chain = false;
      // Do not use a for each loop style.
      // The vector object_relations_ is modified by resolveChain
      for(size_t rel_i = 0; rel_i < indiv->object_relations_.size(); rel_i++)
      {
        std::unordered_set<ObjectPropertyBranch_t*> props = ontology_->object_property_graph_.getUpPtrSafe(indiv->object_relations_[rel_i].first);
        for(ObjectPropertyBranch_t* it_prop : props)
        {
          has_active_chain = has_active_chain || (it_prop->chains_.size() != 0);
          for(auto& chain : it_prop->chains_)
            resolveChain(it_prop, chain, indiv, indiv->object_relations_[rel_i].second);
        }
      }
      // To prevent the chain to be triggered only when the first relation is added
      if(has_active_chain)
        indiv->flags_["chain"] = {};
      else
        indiv->flags_.erase("chain");
    }
}

void ReasonerChain::resolveChain(ObjectPropertyBranch_t* prop, std::vector<ObjectPropertyBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on)
{
  auto indivs_node = new ChainNode_t;
  indivs_node->on_ = on;
  indivs_node->from_ = indiv;
  indivs_node->prop_ = prop;

  ChainTree tree;
  tree.push(nullptr, indivs_node);

  size_t chain_size = chain.size() - 1;
  for(size_t link_i = 0; link_i < chain_size; link_i++)
    resolveLink(chain[link_i], &tree, link_i);

  tree.purge(chain_size);

  std::vector<IndividualBranch_t*> indivs = tree.get(chain_size);
  size_t indivs_size = indivs.size();
  if((chain.size() != 0) && (indivs_size != 0))
    for(size_t i = 0; i < indivs_size; i++)
    {
      if(!relationExists(indiv, chain[chain_size], indivs[i]))
      {
        try {
          ontology_->individual_graph_.addRelation(indiv, chain[chain_size], indivs[i], 1.0, true);
        }
        catch(GraphException& e)
        {
          // We don't notify as we don't consider that as an error but rather as an impossible chain on a given individual
          // notifications_.push_back(std::make_pair(notification_error, "[FAIL][" + std::string(e.what()) + "][add]" + indiv->value() + "|" + chain[chain_size]->value() + "|" + indivs[i]->value()));
          continue;
        }
        
        // Compute explanation about the chain axiom used to infer a new relation
        std::string explanation_reference;
        auto link_chain = tree.getChainTo(indivs[i]);
        for(auto& lc : link_chain)
        {
          if(explanation_reference != "") explanation_reference += ";";
          explanation_reference += lc->toString();
        }
        explanations_.emplace_back("[ADD]" + indiv->value() + "|" + chain[chain_size]->value() + "|" + indivs[i]->value(),
                                   "[ADD]" + explanation_reference);

        indiv->nb_updates_++;
        nb_update_++;

        // Update the object_properties_has_induced field in each individual used for the infered new relation
        for(auto chain_node : link_chain)
        {
          for(size_t relation_index = 0; relation_index < chain_node->from_->object_relations_.size(); relation_index++)
          {
            std::unordered_set<ObjectPropertyBranch_t*> prop_down;
            ontology_->object_property_graph_.getDownPtr(chain_node->prop_, prop_down);
            for(auto& down : prop_down)
              if(chain_node->from_->object_relations_[relation_index].first == down)
                if(chain_node->from_->object_relations_[relation_index].second == chain_node->on_)
                {
                  addInduced(chain_node->from_, relation_index, indiv, chain[chain_size], indivs[i]);
                  break;
                }
          }
        }
      }
    }
}

// Build the ChainTree
void ReasonerChain::resolveLink(ObjectPropertyBranch_t* chain_property, ChainTree* tree, size_t index)
{
  std::unordered_set<std::string> chain_props = ontology_->object_property_graph_.getDown(chain_property->value());

  std::vector<ChainNode_t*> nodes = tree->getNodes(index);
  for(auto& node : nodes)
  {
    for(IndivObjectRelationElement_t& relation : node->on_->object_relations_)
    {
      if(chain_props.find(relation.first->value()) != chain_props.end())
      {
        auto next_node = new ChainNode_t;
        next_node->on_ = relation.second;
        next_node->from_ = node->on_;
        next_node->prop_ = relation.first;
        tree->push(node, next_node);
      }
    }
  }
}

bool ReasonerChain::relationExists(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* chain_prop, IndividualBranch_t* chain_indiv)
{
  for(auto& relation : indiv_on->object_relations_)
  {
    if(relation.second->get() == chain_indiv->get())
    {
      std::unordered_set<ObjectPropertyBranch_t*> down_properties;
      ontology_->object_property_graph_.getDownPtr(chain_prop, down_properties);
      if(down_properties.find(relation.first) != down_properties.end())
        return true;
    }
  }
  return false;
}

void ReasonerChain::addInduced(IndividualBranch_t* indiv, size_t index, IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  // if the induced triplet (from, prop, on) doesn't exist into the  object_property_has_induced field -> push it in the field
  if(indiv->object_relations_.has_induced_object_relations[index]->exist(indiv_from, property, indiv_on) == false)
    indiv->object_relations_.has_induced_object_relations[index]->push(indiv_from, property, indiv_on);
}

std::string ReasonerChain::getName()
{
  return "reasoner chain";
}

std::string ReasonerChain::getDescription()
{
  return "This reasoner resolve the properties chains axioms.";
}

PLUGINLIB_EXPORT_CLASS(ReasonerChain, ReasonerInterface)

} // namespace ontologenius
