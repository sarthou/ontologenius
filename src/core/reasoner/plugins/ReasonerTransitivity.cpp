#include "ontologenius/core/reasoner/plugins/ReasonerTransitivity.h"

#include <cstddef>
#include <map>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  void ReasonerTransitivity::postReason()
  {
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_prop(ontology_->object_property_graph_.mutex_);

    for(auto* indiv : ontology_->individual_graph_.get())
      if(first_run_ ||
         (indiv->isUpdated() && (indiv->same_as_.isUpdated() || indiv->object_relations_.isUpdated())) ||
         (indiv->flags_.find("transi") != indiv->flags_.end()) ||
         indiv->hasUpdatedObjectRelation())
      {
        bool has_active_transitivity = false;
        // /!\ Do not use a for each loop style.
        // /!\ The vector object_relations_ is modified by resolveChain
        for(size_t rel_i = 0; rel_i < indiv->object_relations_.size(); rel_i++)
        {
          auto* base_property = indiv->object_relations_[rel_i].first;
          std::unordered_set<ObjectPropertyBranch*> properties;
          getUpPtrTransitive(base_property, properties);
          for(ObjectPropertyBranch* property : properties)
          {
            has_active_transitivity = true;
            auto end_indivs = resolveChain(indiv->object_relations_[rel_i].second, property, 1);

            if(end_indivs.empty() == false)
            {
              UsedVector local_used;
              if(property != base_property)
                existInInheritance(base_property, property->get(), local_used);
              local_used.emplace_back(indiv->value() + "|" + base_property->value() + "|" + indiv->object_relations_[rel_i].second->value(), indiv->object_relations_.has_induced_object_relations[rel_i]);
              for(auto& used : end_indivs)
              {
                if(ontology_->individual_graph_.relationExists(indiv, property, used.first) == false)
                {
                  int index = -1;
                  try
                  {
                    index = ontology_->individual_graph_.addRelation(indiv, property, used.first, 1.0, true, false);
                    indiv->nb_updates_++;
                  }
                  catch(GraphException& e)
                  {
                    // We don't notify as we don't consider that as an error but rather as an impossible chain on a given individual
                    continue;
                  }

                  used.second.insert(used.second.end(), local_used.begin(), local_used.end());
                  indiv->object_relations_[index].explanation.reserve(used.second.size());
                  for(auto it = used.second.rbegin(); it != used.second.rend(); ++it)
                  {
                    indiv->object_relations_[index].explanation.push_back(it->first);

                    if(it->second->exist(indiv, property, used.first) == false)
                    {
                      it->second->push(indiv, property, used.first);
                      indiv->object_relations_.relations.back().induced_traces.emplace_back(it->second);
                    }
                  }

                  nb_update++;
                  explanations_.emplace_back("[ADD]" + indiv->value() + "|" + property->value() + "|" + used.first->value(),
                                             "[ADD]" + indiv->object_relations_[index].getExplanation());
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

    first_run_ = false;
  }

  void ReasonerTransitivity::getUpPtrTransitive(ObjectPropertyBranch* branch, std::unordered_set<ObjectPropertyBranch*>& res)
  {
    if(branch->properties_.transitive_property_)
      if(res.insert(branch).second == false)
        return;

    for(auto& mother : branch->mothers_)
      getUpPtrTransitive(mother.elem, res);
  }

  std::vector<std::pair<IndividualBranch*, UsedVector>> ReasonerTransitivity::resolveChain(IndividualBranch* indiv, ObjectPropertyBranch* property, size_t current_length)
  {
    std::vector<std::pair<IndividualBranch*, UsedVector>> res;

    if(indiv->same_as_.empty())
    {
      resolveChain(indiv, -1, property, current_length, res);
    }
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
        resolveChain(indiv, (int)i, property, current_length, res);
    }

    if(current_length >= 2)
      res.emplace_back(indiv, UsedVector());

    return res;
  }

  void ReasonerTransitivity::resolveChain(IndividualBranch* indiv, int same_index, ObjectPropertyBranch* property, size_t current_length, std::vector<std::pair<IndividualBranch*, UsedVector>>& res)
  {
    IndividualBranch* individual = indiv;
    if(same_index != -1)
      individual = indiv->same_as_[same_index].elem;

    for(size_t i = 0; i < individual->object_relations_.size(); i++)
    {
      auto* base_property = individual->object_relations_[i].first;
      UsedVector local_used;
      if(existInInheritance(base_property, property->get(), local_used))
      {
        auto down_used = resolveChain(individual->object_relations_[i].second, property, current_length + 1);
        if(down_used.empty() == false)
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