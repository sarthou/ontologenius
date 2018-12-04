#include "ontoloGenius/core/arguer/plugins/ArguerInverseOf.h"
#include <pluginlib/class_list_macros.h>

void ArguerInverseOf::preReason()
{

}

void ArguerInverseOf::postReason()
{
  size_t indiv_i, prop_i, inv_i = 0;
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();
  for(indiv_i = 0; indiv_i < indiv_size; indiv_i++)
    if(indiv[indiv_i]->updated_ == true)
    {
      for(prop_i = 0; prop_i < indiv[indiv_i]->object_properties_name_.size(); prop_i++)
      {
        for(inv_i = 0; inv_i < indiv[indiv_i]->object_properties_name_[prop_i]->inverses_.size(); inv_i++)
        {
          IndividualBranch_t* sub_indiv = indiv[indiv_i]->object_properties_on_[prop_i];
          insetInverse(sub_indiv,
                      indiv[indiv_i]->object_properties_name_[prop_i]->inverses_[inv_i],
                      indiv[indiv_i]);
        }
      }
    }
}

void ArguerInverseOf::insetInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv)
{
  size_t properties_size = indiv_on->object_properties_name_.size();
  for(size_t i = 0; i < properties_size; i++)
  {
    if(indiv_on->object_properties_name_[i] == inv_prop)
      if(indiv_on->object_properties_on_[i] == inv_indiv)
        return;
  }

  indiv_on->object_properties_name_.push_back(inv_prop);
  indiv_on->object_properties_on_.push_back(inv_indiv);
  indiv_on->object_properties_deduced_.push_back(false);
  indiv_on->nb_updates_++;
  nb_update_++;
}

std::string ArguerInverseOf::getName()
{
  return "arguer inverse of";
}

std::string ArguerInverseOf::getDesciption()
{
  return "This arguer creates the inverse properties for each individual.";
}

PLUGINLIB_EXPORT_CLASS(ArguerInverseOf, ArguerInterface)
