#include "ontoloGenius/core/arguer/plugins/ArguerGeneralize.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

void ArguerGeneralize::preReason()
{

}

void ArguerGeneralize::postReason()
{

}

void ArguerGeneralize::periodicReason()
{
  std::vector<IndividualBranch_t*> individuals = ontology_->individual_graph_.get();
  for(auto indiv : individuals)
    if(indiv->updated_ == true)
    {

    }
}

std::string ArguerGeneralize::getName()
{
  return "arguer generalize";
}

std::string ArguerGeneralize::getDesciption()
{
  return "This arguer aims to infer new knowledge by generalizing explicit relationships between concepts.";
}

PLUGINLIB_EXPORT_CLASS(ArguerGeneralize, ArguerInterface)
