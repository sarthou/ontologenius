#include "ontologenius/core/reasoner/plugins/ReasonerSymetric.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerSymetric::preReason()
{

}

void ReasonerSymetric::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  size_t prop_i = 0;
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();
  for(size_t indiv_i = 0; indiv_i < indiv_size; indiv_i++)
  {
    if(indiv[indiv_i]->updated_ == true)
      for(prop_i = 0; prop_i < indiv[indiv_i]->object_relations_.size(); prop_i++)
      {
        if(indiv[indiv_i]->object_relations_[prop_i].first->properties_.symetric_property_ == true)
        {
          IndividualBranch_t* sym_indiv = indiv[indiv_i]->object_relations_[prop_i].second;
          ObjectPropertyBranch_t* sym_prop = indiv[indiv_i]->object_relations_[prop_i].first;
          if(!symetricExist(indiv[indiv_i], sym_prop, sym_indiv))
          {
            try
            {
              int index = ontology_->individual_graph_.addProperty(sym_indiv, sym_prop, indiv[indiv_i], 1.0, true);
              if(index == (int)sym_indiv->object_relations_.size() - 1)
                sym_indiv->object_properties_has_induced_.emplace_back();
              sym_indiv->nb_updates_++;

              explanations_.emplace_back("[ADD]" + sym_indiv->value() + "|" + sym_prop->value() + "|" + indiv[indiv_i]->value(),
                                        "[ADD]" + indiv[indiv_i]->value() + "|" + sym_prop->value() + "|" + sym_indiv->value());
              nb_update_++;
            }
            catch(GraphException& e)
            {
              notifications_.push_back(std::make_pair(notification_error, "[FAIL][" + std::string(e.what()) + "][add]" + sym_indiv->value() + "|" + sym_prop->value() + "|" + indiv[indiv_i]->value()));
            }
          }
        }
      }
  }
}

bool ReasonerSymetric::symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  size_t properties_size = sym_indiv->object_relations_.size();
  for(size_t i = 0; i < properties_size; i++)
  {
    if(sym_indiv->object_relations_[i].first->get() == sym_prop->get())
      if(sym_indiv->object_relations_[i].second->get() == indiv_on->get())
        return true;
  }
  return false;
}

std::string ReasonerSymetric::getName()
{
  return "reasoner symetric";
}

std::string ReasonerSymetric::getDesciption()
{
  return "This reasoner creates the symetric properties for each individual.";
}

PLUGINLIB_EXPORT_CLASS(ReasonerSymetric, ReasonerInterface)

} // namespace ontologenius
