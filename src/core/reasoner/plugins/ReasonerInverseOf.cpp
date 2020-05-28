#include "ontologenius/core/reasoner/plugins/ReasonerInverseOf.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerInverseOf::preReason()
{

}

void ReasonerInverseOf::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  size_t indiv_i = 0;
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();
  for(indiv_i = 0; indiv_i < indiv_size; indiv_i++)
    if(indiv[indiv_i]->updated_ == true)
    {
      for(IndivObjectRelationElement_t& relation : indiv[indiv_i]->object_relations_)
      {
        auto inverts = getLowestInvert(relation.first);
        for(auto& invert : inverts)
        {
          IndividualBranch_t* sub_indiv = relation.second;
          insertInverse(sub_indiv,
                      invert.elem,
                      indiv[indiv_i]);
        }
      }
    }
}

void ReasonerInverseOf::insertInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv)
{
  size_t properties_size = indiv_on->object_relations_.size();
  for(size_t i = 0; i < properties_size; i++)
  {
    auto up_properties = ontology_->object_property_graph_.getUpPtrSafe(indiv_on->object_relations_[i].first);
    if(std::find(up_properties.begin(), up_properties.end(), inv_prop) != up_properties.end())
      if(indiv_on->object_relations_[i].second == inv_indiv)
        return;
  }

  indiv_on->object_relations_.emplace_back(inv_prop, inv_indiv, 1.0, true);
  indiv_on->object_properties_has_induced_.emplace_back();
  indiv_on->nb_updates_++;
  nb_update_++;
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
