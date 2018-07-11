#include "ontoloGenius/core/arguer/plugins/ArguerChain.h"
#include <pluginlib/class_list_macros.h>

void ArguerChain::preReason()
{

}

void ArguerChain::postReason()
{
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();
  for(size_t indiv_i = 0; indiv_i < indiv_size; indiv_i++)
    if(indiv[indiv_i]->updated_ == true)
    {
      for(size_t prop_i = 0; prop_i < indiv[indiv_i]->object_properties_name_.size(); prop_i++)
      {
        std::unordered_set<ObjectPropertyBranch_t*> props = ontology_->object_property_graph_.getUpPtr(indiv[indiv_i]->object_properties_name_[prop_i]);
        for(ObjectPropertyBranch_t* it_prop : props)
          for(size_t chain_i = 0; chain_i < it_prop->chains_.size(); chain_i++)
            resolveChain(it_prop->chains_[chain_i], indiv[indiv_i]->object_properties_on_[prop_i], indiv[indiv_i]);
      }
    }
}

void ArguerChain::resolveChain(std::vector<ObjectPropertyBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on)
{
  std::vector<IndividualBranch_t*> indivs;
  indivs.push_back(indiv);

  size_t chain_size = chain.size() - 1;
  for(size_t link_i = 0; link_i < chain_size; link_i++)
  {
    resolveLink(chain[link_i], indivs);
  }

  size_t indivs_size = indivs.size();
  if((chain.size() != 0) && (indivs_size != 0))
    for(size_t i = 0; i < indivs_size; i++)
      if(!porpertyExist(on, chain[chain_size], indivs[i]))
      {
        on->object_properties_name_.push_back(chain[chain_size]);
        on->object_properties_on_.push_back(indivs[i]);
        on->nb_updates_++;
        nb_update_++;
      }
}

void ArguerChain::resolveLink(ObjectPropertyBranch_t* chain_property, std::vector<IndividualBranch_t*>& indivs)
{
  size_t indiv_i, prop_i = 0;
  std::vector<IndividualBranch_t*> tmp;
  std::unordered_set<std::string> chain_props = ontology_->object_property_graph_.getDown(chain_property->value());

  for(indiv_i = 0; indiv_i < indivs.size(); indiv_i++)
    for(prop_i = 0; prop_i < indivs[indiv_i]->object_properties_name_.size(); prop_i++)
    {
      if(chain_props.find(indivs[indiv_i]->object_properties_name_[prop_i]->value()) != chain_props.end())
        tmp.push_back(indivs[indiv_i]->object_properties_on_[prop_i]);
    }

  indivs = tmp;
}

bool ArguerChain::porpertyExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* chain_prop, IndividualBranch_t* chain_indiv)
{
  size_t properties_size = indiv_on->object_properties_name_.size();
  for(size_t i = 0; i < properties_size; i++)
  {
    if(indiv_on->object_properties_name_[i]->value() == chain_prop->value())
      if(indiv_on->object_properties_on_[i]->value() == chain_indiv->value())
        return true;
  }
  return false;
}

std::string ArguerChain::getName()
{
  return "arguer chain";
}

std::string ArguerChain::getDesciption()
{
  return "This arguer resolve the properties chains axioms.";
}

PLUGINLIB_EXPORT_CLASS(ArguerChain, ArguerInterface)
