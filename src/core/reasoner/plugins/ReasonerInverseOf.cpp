#include "ontologenius/core/reasoner/plugins/ReasonerInverseOf.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerInverseOf::preReason()
{

}

void ReasonerInverseOf::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.get();
  for(auto& indiv : indivs)
    if(indiv->updated_ == true)
    {
      for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
      {
        auto inverts = getLowestInvert(relation.first);
        for(auto& invert : inverts)
        {
          IndividualBranch_t* sub_indiv = relation.second;
          insertInverse(sub_indiv,
                      relation.first,
                      invert.elem,
                      indiv);
        }
      }
    } 
}

void ReasonerInverseOf::insertInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* base_prop, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv)
{
  for(auto& prop : indiv_on->object_relations_)
  {
    if(prop.second == inv_indiv)
    {
      auto up_properties = ontology_->object_property_graph_.getUpPtrSafe(prop.first);
      if(std::find(up_properties.begin(), up_properties.end(), inv_prop) != up_properties.end())
        return;
    }  
  }

  try
  {
    int index = ontology_->individual_graph_.addProperty(indiv_on, inv_prop, inv_indiv, 1.0, true);
    if(index == (int)indiv_on->object_relations_.size() - 1)
      indiv_on->object_properties_has_induced_.emplace_back();
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

std::string ReasonerInverseOf::getDesciption()
{
  return "This reasoner creates the inverse properties for each individual.";
}

PLUGINLIB_EXPORT_CLASS(ReasonerInverseOf, ReasonerInterface)

} // namespace ontologenius
