#include "ontologenius/core/reasoner/plugins/ReasonerTransitive.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerTransitive::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.get();

  
}

bool ReasonerTransitive::transitiveExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* trans_prop, IndividualBranch_t* trans_indiv)
{
  for(auto& relation : trans_indiv->object_relations_)
  {
    if(relation.first->get() == trans_prop->get())
      if(relation.second->get() == indiv_on->get())
        return true;
  }
  return false;
}

std::string ReasonerTransitive::getName()
{
  return "reasoner transitive";
}

std::string ReasonerTransitive::getDescription()
{
  return "This reasoner creates the transitive properties for each individual.";
}

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerTransitive, ontologenius::ReasonerInterface)

} // namespace ontologenius
