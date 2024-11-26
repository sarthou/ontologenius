#include "ontologenius/core/reasoner/plugins/ReasonerInverseOf.h"

#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  void ReasonerInverseOf::postReason()
  {
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
    for(const auto& indiv : ontology_->individual_graph_.get())
      if(indiv->updated_ || indiv->hasUpdatedObjectRelation())
      {
        for(const IndivObjectRelationElement& relation : indiv->object_relations_)
        {
          auto inverts = getLowestInvert(relation.first);
          for(auto& invert : inverts)
          {
            insertInverse(relation.second,
                          relation.first,
                          invert.elem,
                          indiv);
          }
        }
      }
  }

  void ReasonerInverseOf::insertInverse(IndividualBranch* indiv_on, ObjectPropertyBranch* base_prop, ObjectPropertyBranch* inv_prop, IndividualBranch* inv_indiv)
  {
    for(auto it = indiv_on->object_relations_.rbegin(); it != indiv_on->object_relations_.rend(); ++it)
    {
      if(it->second == inv_indiv)
      {
        if(it->first == inv_prop)
          return;

        auto up_properties = ontology_->object_property_graph_.getUpPtrSafe(it->first);
        if(up_properties.find(inv_prop) != up_properties.end())
          return;
      }
    }

    try
    {
      int index = ontology_->individual_graph_.addRelation(indiv_on, inv_prop, inv_indiv, 1.0, true, false);
      indiv_on->object_relations_[index].explanation = {inv_indiv->value() + "|" + base_prop->value() + "|" + indiv_on->value()};
      indiv_on->nb_updates_++;

      explanations_.emplace_back("[ADD]" + indiv_on->value() + "|" + inv_prop->value() + "|" + inv_indiv->value(),
                                 "[ADD]" + indiv_on->object_relations_[index].getExplanation());
      nb_update++;
    }
    catch(GraphException& e)
    {
      notifications_.emplace_back(notification_error, "[FAIL][" + std::string(e.what()) + "][add]" + indiv_on->value() + "|" + inv_prop->value() + "|" + inv_indiv->value());
    }
  }

  std::vector<ObjectPropertyElement> ReasonerInverseOf::getLowestInvert(ObjectPropertyBranch* base_prop)
  {
    if(base_prop->inverses_.empty() == false)
      return base_prop->inverses_;
    else
    {
      std::vector<ObjectPropertyElement> res;

      for(auto& up : base_prop->mothers_)
      {
        auto tmp = getLowestInvert(up.elem);
        if(tmp.empty() == false)
          res.insert(res.end(), tmp.begin(), tmp.end());
      }

      return res;
    }
  }

  std::string ReasonerInverseOf::getName()
  {
    return "reasoner inverse of";
  }

  std::string ReasonerInverseOf::getDescription()
  {
    return "This reasoner creates the inverse properties for each individual.";
  }

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerInverseOf, ontologenius::ReasonerInterface)
