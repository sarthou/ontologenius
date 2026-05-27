#include "ontologenius/core/reasoner/plugins/ReasonerSymmetric.h"

#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_set>

#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  void ReasonerSymmetric::postReason()
  {
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individuals_.mutex_);
    // not impacted by same as
    for(const auto& indiv : ontology_->individuals_.get())
    {
      if(first_run_ ||
         (indiv->isUpdated() && (indiv->same_as_.isUpdated() || indiv->object_relations_.isUpdated())) ||
         indiv->hasUpdatedObjectRelation())
        for(auto& relation : indiv->object_relations_)
        {
          if(relation.first->properties_.symetric_property_ == true)
            addSymetricRelation(indiv, relation.first, relation.second);
          else
            runOnHierarchy(indiv, relation.first, relation.second);
        }
    }

    first_run_ = false;
  }

  void ReasonerSymmetric::runOnHierarchy(IndividualBranch* first_indiv, ObjectPropertyBranch* property, IndividualBranch* second_indiv)
  {
    for(auto& up_property : property->mothers_)
    {
      if(up_property.elem->properties_.symetric_property_ == true)
        addSymetricRelation(first_indiv, up_property.elem, second_indiv);
      else
        runOnHierarchy(first_indiv, up_property.elem, second_indiv);
    }
  }

  void ReasonerSymmetric::addSymetricRelation(IndividualBranch* indiv_on, ObjectPropertyBranch* sym_prop, IndividualBranch* sym_indiv)
  {
    if(!symetricExist(indiv_on, sym_prop, sym_indiv))
    {
      try
      {
        int index = ontology_->individuals_.addRelation(sym_indiv, sym_prop, indiv_on, 1.0, true, false);
        sym_indiv->object_relations_[index].explanation = {indiv_on->value() + "|" + sym_prop->value() + "|" + sym_indiv->value()};
        sym_indiv->nb_updates_++;

        explanations_.emplace_back("[ADD]" + sym_indiv->value() + "|" + sym_prop->value() + "|" + indiv_on->value(),
                                   "[ADD]" + sym_indiv->object_relations_[index].getExplanation());
        nb_update++;
      }
      catch(GraphException& e)
      {
        notifications_.emplace_back(notification_error, "[FAIL][" + std::string(e.what()) + "][add]" + sym_indiv->value() + "|" + sym_prop->value() + "|" + indiv_on->value());
      }
    }
  }

  bool ReasonerSymmetric::symetricExist(IndividualBranch* indiv_on, ObjectPropertyBranch* sym_prop, IndividualBranch* sym_indiv)
  {
    for(auto& relation : sym_indiv->object_relations_)
    {
      std::unordered_set<ObjectPropertyBranch*> down_properties;
      ontology_->object_properties_.getDownPtr(sym_prop, down_properties);
      if(down_properties.count(relation.first) != 0)
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