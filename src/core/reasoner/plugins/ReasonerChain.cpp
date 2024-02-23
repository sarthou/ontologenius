#include "ontologenius/core/reasoner/plugins/ReasonerChain.h"

#include <pluginlib/class_list_macros.hpp>

namespace ontologenius { // std::unordered_set::~unordered_set

void ReasonerChain::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::lock_guard<std::shared_timed_mutex> lock_prop(ontology_->object_property_graph_.mutex_);

  for(auto indiv : ontology_->individual_graph_.get())
    if((indiv->updated_ == true) || (indiv->flags_.find("chain") != indiv->flags_.end()) || indiv->hasUpdatedObjectRelation())
    {
      bool has_active_chain = false;
      // /!\ Do not use a for each loop style.
      // /!\ The vector object_relations_ is modified by resolveChain
      for(size_t rel_i = 0; rel_i < indiv->object_relations_.size(); rel_i++)
      {
        auto base_property = indiv->object_relations_[rel_i].first;
        std::unordered_set<ObjectPropertyBranch_t*> properties;
        getUpPtrChain(base_property, properties);
        for(ObjectPropertyBranch_t* property : properties)
        {
          has_active_chain = true;
          for(auto& chain : property->chains_)
          {
            auto end_indivs = resolveChain(indiv->object_relations_[rel_i].second, chain, 0);

            if(end_indivs.size())
            {
              UsedVector local_used;
              if(property != base_property)
                existInInheritance(base_property, property->get(), local_used);
              local_used.emplace_back(indiv->value() + "|" + base_property->value() + "|" + indiv->object_relations_[rel_i].second->value(), indiv->object_relations_.has_induced_object_relations[rel_i]);
              for(auto& used : end_indivs)
              {
                if(!relationExists(indiv, chain.back(), used.first))
                {
                  try {
                    ontology_->individual_graph_.addRelation(indiv, chain.back(), used.first, 1.0, true, false);
                    indiv->nb_updates_++;
                  }
                  catch(GraphException& e)
                  {
                    // We don't notify as we don't consider that as an error but rather as an impossible chain on a given individual
                    continue;
                  }

                  used.second.insert(used.second.end(), local_used.begin(), local_used.end());
                  std::string explanation_reference = "";
                  for(auto it = used.second.rbegin(); it != used.second.rend(); ++it)
                  {
                    if(explanation_reference != "") explanation_reference += ", ";
                    explanation_reference += it->first;

                    if(it->second->exist(indiv, chain.back(), used.first) == false)
                    {
                      it->second->push(indiv, chain.back(), used.first);
                      indiv->object_relations_.relations.back().induced_traces.emplace_back(it->second);
                    }
                  }

                  nb_update_++;
                  explanations_.emplace_back("[ADD]" + indiv->value() + "|" + chain.back()->value() + "|" + used.first->value(),
                                              "[ADD]" + explanation_reference);
                }
              }
            }
          }
        }
      }

      // To prevent the chain to be triggered only when the first relation is added
      if(has_active_chain)
        indiv->flags_["chain"] = {};
      else
        indiv->flags_.erase("chain");
    }
}

void ReasonerChain::getUpPtrChain(ObjectPropertyBranch_t* branch, std::unordered_set<ObjectPropertyBranch_t*>& res)
{
  if(branch->chains_.size())
    if(res.insert(branch).second == false)
      return;

  for(auto& mother : branch->mothers_)
    getUpPtrChain(mother.elem, res);
}

std::vector<std::pair<IndividualBranch_t*, UsedVector>> ReasonerChain::resolveChain(IndividualBranch_t* indiv, const std::vector<ObjectPropertyBranch_t*>& chain, size_t chain_index)
{
  if(chain_index >= chain.size() - 1)
    return {std::make_pair(indiv, UsedVector())};
  else
  {
    std::vector<std::pair<IndividualBranch_t*, UsedVector>> res;
    if(indiv->same_as_.empty())
      resolveChain(indiv, -1, chain, chain_index, res);
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
        resolveChain(indiv, i, chain, chain_index, res);
    }

    return res;
  }
}

void ReasonerChain::resolveChain(IndividualBranch_t* indiv, int same_index, const std::vector<ObjectPropertyBranch_t*>& chain, size_t chain_index, std::vector<std::pair<IndividualBranch_t*, UsedVector>>& res)
{
  IndividualBranch_t* individual = indiv;
  if(same_index != -1)
    individual = indiv->same_as_[same_index].elem;

  for(size_t i = 0; i < individual->object_relations_.size(); i++)
  {
    auto base_property = individual->object_relations_[i].first;
    UsedVector local_used;
    if(existInInheritance(base_property, chain[chain_index]->get(), local_used))
    {
      auto down_used = resolveChain(individual->object_relations_[i].second, chain, chain_index + 1);
      if(down_used.size())
      {
        local_used.emplace_back(individual->value() + "|" + base_property->value() + "|" + individual->object_relations_[i].second->value(), individual->object_relations_.has_induced_object_relations[i]);
        if((same_index != -1) && (individual != indiv))
          local_used.emplace_back(indiv->value() + "|sameAs|" + individual->value(), indiv->same_as_.has_induced_object_relations[same_index]);
        for(auto& used : down_used)
        {
          res.push_back(used);
          res.back().second.insert(res.back().second.end(), local_used.begin(), local_used.end());
        }
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

std::string ReasonerChain::getName()
{
  return "reasoner chain";
}

std::string ReasonerChain::getDescription()
{
  return "This reasoner resolve the properties chains axioms.";
}

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerChain, ontologenius::ReasonerInterface)