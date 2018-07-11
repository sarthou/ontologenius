#include "ontoloGenius/core/arguer/plugins/ArguerSymetric.h"
#include <pluginlib/class_list_macros.h>

void ArguerSymetric::preReason()
{

}

void ArguerSymetric::postReason()
{
  size_t prop_i = 0;
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();
  for(size_t indiv_i = 0; indiv_i < indiv_size; indiv_i++)
  {
    if(indiv[indiv_i]->updated_ == true)
      for(prop_i = 0; prop_i < indiv[indiv_i]->object_properties_name_.size(); prop_i++)
      {
        if(indiv[indiv_i]->object_properties_name_[prop_i]->properties_.symetric_property_ == true)
        {
          IndividualBranch_t* sym_indiv = indiv[indiv_i]->object_properties_on_[prop_i];
          ObjectPropertyBranch_t* sym_prop = indiv[indiv_i]->object_properties_name_[prop_i];
          if(!symetricExist(indiv[indiv_i], sym_prop, sym_indiv))
          {
            sym_indiv->object_properties_name_.push_back(sym_prop);
            sym_indiv->object_properties_on_.push_back(indiv[indiv_i]);
            sym_indiv->nb_updates_++;
            nb_update_++;
          }
        }
      }
  }
}

bool ArguerSymetric::symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  size_t properties_size = sym_indiv->object_properties_name_.size();
  for(size_t i = 0; i < properties_size; i++)
  {
    if(sym_indiv->object_properties_name_[i]->value() == sym_prop->value())
      if(sym_indiv->object_properties_on_[i]->value() == indiv_on->value())
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
