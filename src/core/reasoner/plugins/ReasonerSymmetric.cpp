#include "ontologenius/core/reasoner/plugins/ReasonerSymmetric.h"

#include <pluginlib/class_list_macros.hpp>

namespace ontologenius {

void ReasonerSymmetric::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  // not impacted by same as
  for(auto& indiv : ontology_->individual_graph_.get())
  {
    if(indiv->updated_ == true || indiv->hasUpdatedObjectRelation())
      for(auto& relation : indiv->object_relations_)
      {
        if(relation.first->properties_.symetric_property_ == true)
        {
          IndividualBranch_t* sym_indiv = relation.second;
          ObjectPropertyBranch_t* sym_prop = relation.first;
          if(!symetricExist(indiv, sym_prop, sym_indiv))
          {
            try
            {
              ontology_->individual_graph_.addRelation(sym_indiv, sym_prop, indiv, 1.0, true, false);
              sym_indiv->nb_updates_++;

              explanations_.emplace_back("[ADD]" + sym_indiv->value() + "|" + sym_prop->value() + "|" + indiv->value(),
                                        "[ADD]" + indiv->value() + "|" + sym_prop->value() + "|" + sym_indiv->value());
              nb_update_++;
            }
            catch(GraphException& e)
            {
              notifications_.push_back(std::make_pair(notification_error, "[FAIL][" + std::string(e.what()) + "][add]" + sym_indiv->value() + "|" + sym_prop->value() + "|" + indiv->value()));
            }
          }
        }
      }
  }
}

bool ReasonerSymmetric::symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  for(auto& relation : sym_indiv->object_relations_)
  {
    if(relation.first->get() == sym_prop->get())
      if(relation.second->get() == indiv_on->get())
        return true;
  }
  return false;
}

std::string ReasonerSymmetric::getName()
{
  return "reasoner symetric";
}

std::string ReasonerSymmetric::getDescription()
{
  return "This reasoner creates the symetric properties for each individual.";
}

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerSymmetric, ontologenius::ReasonerInterface)