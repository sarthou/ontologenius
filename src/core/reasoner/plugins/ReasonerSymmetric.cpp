#include "ontologenius/core/reasoner/plugins/ReasonerSymmetric.h"

#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>

#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  void ReasonerSymmetric::postReason()
  {
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
    // not impacted by same as
    for(const auto& indiv : ontology_->individual_graph_.get())
    {
      if(indiv->updated_ == true || indiv->hasUpdatedObjectRelation())
        for(auto& relation : indiv->object_relations_)
        {
          if(relation.first->properties_.symetric_property_ == true)
          {
            IndividualBranch* sym_indiv = relation.second;
            ObjectPropertyBranch* sym_prop = relation.first;
            if(!symetricExist(indiv, sym_prop, sym_indiv))
            {
              try
              {
                ontology_->individual_graph_.addRelation(sym_indiv, sym_prop, indiv, 1.0, true, false);
                sym_indiv->nb_updates_++;

                explanations_.emplace_back("[ADD]" + sym_indiv->value() + "|" + sym_prop->value() + "|" + indiv->value(),
                                           "[ADD]" + indiv->value() + "|" + sym_prop->value() + "|" + sym_indiv->value());
                nb_update++;
              }
              catch(GraphException& e)
              {
                notifications_.emplace_back(notification_error, "[FAIL][" + std::string(e.what()) + "][add]" + sym_indiv->value() + "|" + sym_prop->value() + "|" + indiv->value());
              }
            }
          }
        }
    }
  }

  bool ReasonerSymmetric::symetricExist(IndividualBranch* indiv_on, ObjectPropertyBranch* sym_prop, IndividualBranch* sym_indiv)
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