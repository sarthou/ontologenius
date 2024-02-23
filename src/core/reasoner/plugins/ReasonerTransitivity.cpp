#include "ontologenius/core/reasoner/plugins/ReasonerTransitivity.h"

#include <pluginlib/class_list_macros.hpp>

namespace ontologenius {

void ReasonerTransitivity::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::lock_guard<std::shared_timed_mutex> lock_prop(ontology_->object_property_graph_.mutex_);

  for(auto indiv : ontology_->individual_graph_.get())
    if((indiv->updated_ == true) || (indiv->flags_.find("transi") != indiv->flags_.end()) || indiv->hasUpdatedObjectRelation())
    {
      bool has_active_transitivity = false;
      // /!\ Do not use a for each loop style.
      // /!\ The vector object_relations_ is modified by resolveChain
      for(size_t rel_i = 0; rel_i < indiv->object_relations_.size(); rel_i++)
      {
        auto base_property = indiv->object_relations_[rel_i].first;
        std::unordered_set<ObjectPropertyBranch_t*> properties;
        getUpPtrTransitive(base_property, properties);
        for(ObjectPropertyBranch_t* property : properties)
        {
          has_active_transitivity = true;
          auto end_indivs = resolveChain(indiv->object_relations_[rel_i].second, property, 1);

          if(end_indivs.size())
          {
            UsedVector local_used;
            if(property != base_property)
              existInInheritance(base_property, property->get(), local_used);
            local_used.emplace_back(indiv->value() + "|" + base_property->value() + "|" + indiv->object_relations_[rel_i].second->value(), indiv->object_relations_.has_induced_object_relations[rel_i]);
            for(auto& used : end_indivs)
            {
              if(!relationExists(indiv, property, used.first))
              {
                try {
                  ontology_->individual_graph_.addRelation(indiv, property, used.first, 1.0, true, false);
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

                  if(it->second->exist(indiv, property, used.first) == false)
                  {
                    it->second->push(indiv, property, used.first);
                    indiv->object_relations_.relations.back().induced_traces.emplace_back(it->second);
                  }
                }

                nb_update_++;
                explanations_.emplace_back("[ADD]" + indiv->value() + "|" + property->value() + "|" + used.first->value(),
                                            "[ADD]" + explanation_reference);
              }
            }
          }
        }
      }

      // To prevent the transitivity chain to be triggered only when the first relation is added
      if(has_active_transitivity)
        indiv->flags_["transi"] = {};
      else
        indiv->flags_.erase("transi");
    }
}

void ReasonerTransitivity::getUpPtrTransitive(ObjectPropertyBranch_t* branch, std::unordered_set<ObjectPropertyBranch_t*>& res)
{
  if(branch->properties_.transitive_property_)
    if(res.insert(branch).second == false)
      return;

  for(auto& mother : branch->mothers_)
    getUpPtrTransitive(mother.elem, res);
}

std::vector<std::pair<IndividualBranch_t*, UsedVector>> ReasonerTransitivity::resolveChain(IndividualBranch_t* indiv, ObjectPropertyBranch_t* property, size_t current_length)
{
  std::vector<std::pair<IndividualBranch_t*, UsedVector>> res;

  if(indiv->same_as_.empty())
  {
    resolveChain(indiv, -1, property, current_length, res);
  }
  else
  {
    for(size_t i = 0; i < indiv->same_as_.size(); i++)
      resolveChain(indiv, i, property, current_length, res);
  }

  if(current_length >= 2)
    res.emplace_back(indiv, UsedVector());

  return res;
}

void ReasonerTransitivity::resolveChain(IndividualBranch_t* indiv, int same_index, ObjectPropertyBranch_t* property, size_t current_length, std::vector<std::pair<IndividualBranch_t*, UsedVector>>& res)
{
  IndividualBranch_t* individual = indiv;
  if(same_index != -1)
    individual = indiv->same_as_[same_index].elem;

  for(size_t i = 0; i < individual->object_relations_.size(); i++)
  {
    auto base_property = individual->object_relations_[i].first;
    UsedVector local_used;
    if(existInInheritance(base_property, property->get(), local_used))
    {
      auto down_used = resolveChain(individual->object_relations_[i].second, property, current_length + 1);
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

bool ReasonerTransitivity::relationExists(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  for(auto& relation : indiv_from->object_relations_)
  {
    if(relation.second->get() == indiv_on->get())
    {
      std::unordered_set<ObjectPropertyBranch_t*> down_properties;
      ontology_->object_property_graph_.getDownPtr(property, down_properties);
      if(down_properties.find(relation.first) != down_properties.end())
        return true;
    }
  }
  return false;
}

std::string ReasonerTransitivity::getName()
{
  return "reasoner transitivity";
}

std::string ReasonerTransitivity::getDescription()
{
  return "This reasoner resolve transitive axioms.";
}

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerTransitivity, ontologenius::ReasonerInterface)