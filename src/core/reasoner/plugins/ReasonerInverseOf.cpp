#include "ontologenius/core/reasoner/plugins/ReasonerInverseOf.h"

#include <pluginlib/class_list_macros.hpp>

namespace ontologenius {

void ReasonerInverseOf::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  for(auto& indiv : ontology_->individual_graph_.get())
    if(indiv->updated_ || indiv->hasUpdatedObjectRelation())
    {
      for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
      {
        auto inverts = getLowestInvert(relation.first);
        for(auto& invert : inverts)
        {
          insertInverse(relation.second,
                      relation.first,
                      invert.elem,
                      indiv);
        }
      }
    } 
}

void ReasonerInverseOf::insertInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* base_prop, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv)
{
  for(auto it = indiv_on->object_relations_.rbegin(); it != indiv_on->object_relations_.rend(); ++it)
  {
    if(it->second == inv_indiv)
    {
      if(it->first == inv_prop)
        return;
        
      auto up_properties = ontology_->object_property_graph_.getUpPtrSafe(it->first);
      if(std::find(up_properties.begin(), up_properties.end(), inv_prop) != up_properties.end())
        return;
    }  
  }

  try
  {
    ontology_->individual_graph_.addRelation(indiv_on, inv_prop, inv_indiv, 1.0, true, false);
    indiv_on->nb_updates_++;

    explanations_.emplace_back("[ADD]" + indiv_on->value() + "|" + inv_prop->value() + "|" + inv_indiv->value(),
                              "[ADD]" + inv_indiv->value() + "|" + base_prop->value() + "|" + indiv_on->value());
    nb_update_++;
  }
  catch(GraphException& e)
  {
    notifications_.push_back(std::make_pair(notification_error, "[FAIL][" + std::string(e.what()) + "][add]" + indiv_on->value() + "|" + inv_prop->value() + "|" + inv_indiv->value()));
  }
}

std::vector<ObjectPropertyElement_t> ReasonerInverseOf::getLowestInvert(ObjectPropertyBranch_t* base_prop)
{
  if(base_prop->inverses_.size())
    return base_prop->inverses_;
  else
  {
    std::vector<ObjectPropertyElement_t> res;

    for(auto& up : base_prop->mothers_)
    {
      auto tmp = getLowestInvert(up.elem);
      if(tmp.size())
        res.insert(res.end(), tmp.begin(), tmp.end());
    }

    return res;
  }
}

std::string ReasonerInverseOf::getName()
{
  return "reasoner inverse of";
}

std::string ReasonerInverseOf::getDescription()
{
  return "This reasoner creates the inverse properties for each individual.";
}

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerInverseOf, ontologenius::ReasonerInterface)
