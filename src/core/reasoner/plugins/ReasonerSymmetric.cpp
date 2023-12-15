#include "ontologenius/core/reasoner/plugins/ReasonerSymmetric.h"

#if ROS_VERSION == 1
#include <pluginlib/class_list_macros.h>
#elif ROS_VERSION == 2
#include <pluginlib/class_list_macros.hpp>
#endif

namespace ontologenius {

void ReasonerSymmetric::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.get();
  for(auto& indiv : indivs)
  {
    if(indiv->updated_ == true)
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
              int index = ontology_->individual_graph_.addRelation(sym_indiv, sym_prop, indiv, 1.0, true);
              if(index == (int)sym_indiv->object_relations_.size() - 1)
                sym_indiv->object_properties_has_induced_.emplace_back();
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

std::string ReasonerSymmetric::getDesciption()
{
  return "This reasoner creates the symetric properties for each individual.";
}

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerSymmetric, ontologenius::ReasonerInterface)

} // namespace ontologenius
