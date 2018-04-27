#include "ontoloGenius/arguer/plugins/ArguerInverseOf.h"
#include <pluginlib/class_list_macros.h>

void ArguerInverseOf::preReason()
{

}

void ArguerInverseOf::postReason()
{
  std::vector<IndividualBranch_t*> indiv = ontology_->individuals_.get();
  for(size_t indiv_i = 0; indiv_i < indiv.size(); indiv_i++)
  {
    for(size_t prop_i = 0; prop_i < indiv[indiv_i]->properties_name_.size(); prop_i++)
    {
      for(size_t inv_i = 0; inv_i < indiv[indiv_i]->properties_name_[prop_i]->inverses_.size(); inv_i++)
      {
        IndividualBranch_t* sub_indiv = indiv[indiv_i]->properties_on_[prop_i];
        insetInverse(sub_indiv,
                    indiv[indiv_i]->properties_name_[prop_i]->inverses_[inv_i],
                    indiv[indiv_i]);
      }
    }
  }
}

void ArguerInverseOf::insetInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv)
{
  for(size_t i = 0; i < indiv_on->properties_name_.size(); i++)
  {
    if(indiv_on->properties_name_[i] == inv_prop)
      if(indiv_on->properties_on_[i] == inv_indiv)
        return;
  }

  indiv_on->properties_name_.push_back(inv_prop);
  indiv_on->properties_on_.push_back(inv_indiv);
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
