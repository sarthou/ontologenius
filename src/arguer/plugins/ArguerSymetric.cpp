#include "ontoloGenius/arguer/plugins/ArguerSymetric.h"
#include <pluginlib/class_list_macros.h>

void ArguerSymetric::preReason()
{

}

void ArguerSymetric::postReason()
{
  std::vector<IndividualBranch_t*> indiv = ontology_->individuals_.get();
  for(size_t indiv_i = 0; indiv_i < indiv.size(); indiv_i++)
  {
    for(size_t prop_i = 0; prop_i < indiv[indiv_i]->properties_name_.size(); prop_i++)
    {
      if(indiv[indiv_i]->properties_name_[prop_i]->properties_.symetric_property_ == true)
      {
        IndividualBranch_t* sym_indiv = indiv[indiv_i]->properties_on_[prop_i];
        PropertyClassBranch_t* sym_prop = indiv[indiv_i]->properties_name_[prop_i];
        if(!symetricExist(indiv[indiv_i], sym_prop, sym_indiv))
        {
          sym_indiv->properties_name_.push_back(sym_prop);
          sym_indiv->properties_on_.push_back(indiv[indiv_i]);
        }
      }
    }
  }
}

bool ArguerSymetric::symetricExist(IndividualBranch_t* indiv_on, PropertyClassBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  for(size_t i = 0; i < sym_indiv->properties_name_.size(); i++)
  {
    if(sym_indiv->properties_name_[i]->value_ == sym_prop->value_)
      if(sym_indiv->properties_on_[i]->value_ == indiv_on->value_)
        return true;
  }
  return false;
}

std::string ArguerSymetric::getName()
{
  return "arguer symetric";
}

std::string ArguerSymetric::getDesciption()
{
  return "This arguer creates the symetric properties for each individual.";
}

PLUGINLIB_EXPORT_CLASS(ArguerSymetric, ArguerInterface)
