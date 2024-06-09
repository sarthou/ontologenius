#include "ontologenius/core/reasoner/plugins/ReasonerRangeDomain.h"

#include <map>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  void ReasonerRangeDomain::postReason()
  {
    // flags "range" and "domain" come from the checkers marking warnings and on relation insertion
    postReasonIndividuals();
    postReasonClasses();
  }

  void ReasonerRangeDomain::postReasonIndividuals()
  {
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);

    std::map<std::string, std::vector<std::string>>::iterator it_range;
    std::map<std::string, std::vector<std::string>>::iterator it_domain;

    for(const auto& indiv : ontology_->individual_graph_.get())
      if(indiv->updated_ == true || indiv->hasUpdatedObjectRelation() || indiv->hasUpdatedDataRelation())
      {
        it_range = indiv->flags_.find("range");
        if(it_range != indiv->flags_.end())
        {
          for(const std::string& prop : it_range->second)
            deduceRange(indiv, prop);
          indiv->flags_.erase("range");
        }

        it_domain = indiv->flags_.find("domain");
        if(it_domain != indiv->flags_.end())
        {
          for(const std::string& prop : it_domain->second)
            deduceDomain(indiv, prop);
          indiv->flags_.erase("domain");
        }
      }
  }

  void ReasonerRangeDomain::deduceRange(IndividualBranch* branch, const std::string& prop)
  {
    for(auto& relation : branch->object_relations_)
      if(relation.first->value() == prop)
        deduceObjRange(relation);
  }

  void ReasonerRangeDomain::deduceDomain(IndividualBranch* branch, const std::string& prop)
  {
    for(auto& relation : branch->object_relations_)
      if(relation.first->value() == prop)
        deduceObjDomain(branch, relation);

    for(auto& relation : branch->data_relations_)
      if(relation.first->value() == prop)
        deduceDatDomain(branch, relation);
  }

  void ReasonerRangeDomain::deduceObjRange(IndivObjectRelationElement& relation)
  {
    std::unordered_set<ClassBranch*> ranges;
    std::unordered_set<ObjectPropertyBranch*> props = {relation.first};
    while(ranges.empty())
    {
      for(auto* prop : props)
        ontology_->object_property_graph_.getRangePtr(prop, ranges, 0);

      if(ranges.empty())
      {
        std::unordered_set<ObjectPropertyBranch*> prop_up;
        for(auto* prop : props)
        {
          ontology_->object_property_graph_.getUpPtr(prop, prop_up, 1);
          prop_up.erase(prop);
        }

        if(prop_up.empty())
          break;
        else
          props.swap(prop_up);
      }
    }

    for(auto* range : ranges)
    {
      std::unordered_set<ClassBranch*> up;
      if(relation.second->same_as_.empty())
        ontology_->individual_graph_.getUpPtr(relation.second, up);
      else
      {
        for(auto& same : relation.second->same_as_.relations)
          ontology_->individual_graph_.getUpPtr(same.elem, up);
      }

      if(up.find(range) == up.end())
      {
        relation.second->is_a_.emplaceBack(range, 1.0, true);
        range->individual_childs_.emplace_back(relation.second, 1.0, true);

        relation.second->nb_updates_++;
        range->nb_updates_++;
        nb_update++;
      }
    }
  }

  void ReasonerRangeDomain::deduceObjDomain(IndividualBranch* branch, IndivObjectRelationElement& relation)
  {
    std::unordered_set<ClassBranch*> domains;
    std::unordered_set<ObjectPropertyBranch*> props = {relation.first};
    while(domains.empty())
    {
      for(auto* prop : props)
        ontology_->object_property_graph_.getDomainPtr(prop, domains, 0);

      if(domains.empty())
      {
        std::unordered_set<ObjectPropertyBranch*> prop_up;
        for(auto* prop : props)
        {
          ontology_->object_property_graph_.getUpPtr(prop, prop_up, 1);
          prop_up.erase(prop);
        }

        if(prop_up.empty())
          break;
        else
          props.swap(prop_up);
      }
    }

    for(auto* domain : domains)
    {
      std::unordered_set<ClassBranch*> up;
      if(branch->same_as_.empty())
        ontology_->individual_graph_.getUpPtr(branch, up);
      else
      {
        for(auto& same : branch->same_as_.relations)
          ontology_->individual_graph_.getUpPtr(same.elem, up);
      }

      if(up.find(domain) == up.end())
      {
        branch->is_a_.emplaceBack(domain, 1.0, true);
        domain->individual_childs_.emplace_back(branch, 1.0, true);

        branch->nb_updates_++;
        domain->nb_updates_++;
        nb_update++;
      }
    }
  }

  void ReasonerRangeDomain::deduceDatDomain(IndividualBranch* branch, IndivDataRelationElement& relation)
  {
    std::unordered_set<ClassBranch*> domains;
    std::unordered_set<DataPropertyBranch*> props = {relation.first};
    while(domains.empty())
    {
      for(auto* prop : props)
        ontology_->data_property_graph_.getDomainPtr(prop, domains, 0);

      if(domains.empty())
      {
        std::unordered_set<DataPropertyBranch*> prop_up;
        for(auto* prop : props)
        {
          ontology_->data_property_graph_.getUpPtr(prop, prop_up, 1);
          prop_up.erase(prop);
        }

        if(prop_up.empty())
          break;
        else
          props.swap(prop_up);
      }
    }

    for(auto* domain : domains)
    {
      std::unordered_set<ClassBranch*> up;
      if(branch->same_as_.empty())
        ontology_->individual_graph_.getUpPtr(branch, up);
      else
      {
        for(auto& same : branch->same_as_.relations)
          ontology_->individual_graph_.getUpPtr(same.elem, up);
      }

      if(up.find(domain) == up.end())
      {
        branch->is_a_.emplaceBack(domain, 1.0, true);
        domain->individual_childs_.emplace_back(branch, 1.0, true);

        branch->nb_updates_++;
        domain->nb_updates_++;
        nb_update++;
      }
    }
  }

  void ReasonerRangeDomain::postReasonClasses()
  {
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->class_graph_.mutex_);
    const std::vector<ClassBranch*> classes = ontology_->class_graph_.get();

    std::map<std::string, std::vector<std::string>>::iterator it_range;
    std::map<std::string, std::vector<std::string>>::iterator it_domain;

    for(auto* class_branch : classes)
      if(class_branch->updated_ == true)
      {
        it_range = class_branch->flags_.find("range");
        if(it_range != class_branch->flags_.end())
        {
          for(const std::string& prop : it_range->second)
            deduceRange(class_branch, prop);
          class_branch->flags_.erase("range");
        }

        it_domain = class_branch->flags_.find("domain");
        if(it_domain != class_branch->flags_.end())
        {
          for(const std::string& prop : it_domain->second)
            deduceDomain(class_branch, prop);
          class_branch->flags_.erase("domain");
        }
      }
  }

  void ReasonerRangeDomain::deduceRange(ClassBranch* branch, const std::string& prop)
  {
    for(auto& relation : branch->object_relations_)
      if(relation.first->value() == prop)
        deduceObjRange(relation);
  }

  void ReasonerRangeDomain::deduceDomain(ClassBranch* branch, const std::string& prop)
  {
    for(auto& relation : branch->object_relations_)
      if(relation.first->value() == prop)
        deduceObjDomain(branch, relation);

    for(auto& relation : branch->data_relations_)
      if(relation.first->value() == prop)
        deduceDatDomain(branch, relation);
  }

  void ReasonerRangeDomain::deduceObjRange(ClassObjectRelationElement& relation)
  {
    std::unordered_set<ClassBranch*> ranges;
    std::unordered_set<ObjectPropertyBranch*> props = {relation.first};
    while(ranges.empty())
    {
      for(auto* prop : props)
        ontology_->object_property_graph_.getRangePtr(prop, ranges, 0);

      if(ranges.empty())
      {
        std::unordered_set<ObjectPropertyBranch*> prop_up;
        for(auto* prop : props)
        {
          ontology_->object_property_graph_.getUpPtr(prop, prop_up, 1);
          prop_up.erase(prop);
        }

        if(prop_up.empty())
          break;
        else
          props = prop_up;
      }
    }

    for(auto* range : ranges)
    {
      std::unordered_set<ClassBranch*> up;
      ontology_->class_graph_.getUpPtr(relation.second, up);
      if(up.find(range) == up.end())
      {
        relation.second->mothers_.emplaceBack(range, 1.0, true);
        range->childs_.emplace_back(relation.second, 1.0, true);

        relation.second->nb_updates_++;
        range->nb_updates_++;
        nb_update++;
      }
    }
  }

  void ReasonerRangeDomain::deduceObjDomain(ClassBranch* branch, ClassObjectRelationElement& relation)
  {
    std::unordered_set<ClassBranch*> domains;
    std::unordered_set<ObjectPropertyBranch*> props = {relation.first};
    while(domains.empty())
    {
      for(auto* prop : props)
        ontology_->object_property_graph_.getDomainPtr(prop, domains, 0);

      if(domains.empty())
      {
        std::unordered_set<ObjectPropertyBranch*> prop_up;
        for(auto* prop : props)
        {
          ontology_->object_property_graph_.getUpPtr(prop, prop_up, 1);
          prop_up.erase(prop);
        }

        if(prop_up.empty())
          break;
        else
          props.swap(prop_up);
      }
    }

    for(auto* domain : domains)
    {
      std::unordered_set<ClassBranch*> up;
      ontology_->class_graph_.getUpPtr(branch, up);
      if(up.find(domain) == up.end())
      {
        branch->mothers_.emplaceBack(domain, 1.0, true);
        domain->childs_.emplace_back(branch, 1.0, true);

        branch->nb_updates_++;
        domain->nb_updates_++;
        nb_update++;
      }
    }
  }

  void ReasonerRangeDomain::deduceDatDomain(ClassBranch* branch, ClassDataRelationElement& relation)
  {
    std::unordered_set<ClassBranch*> domains;
    std::unordered_set<DataPropertyBranch*> props = {relation.first};
    while(domains.empty())
    {
      for(auto* prop : props)
        ontology_->data_property_graph_.getDomainPtr(prop, domains, 0);

      if(domains.empty())
      {
        std::unordered_set<DataPropertyBranch*> prop_up;
        for(auto* prop : props)
        {
          ontology_->data_property_graph_.getUpPtr(prop, prop_up, 1);
          prop_up.erase(prop);
        }

        if(prop_up.empty())
          break;
        else
          props.swap(prop_up);
      }
    }

    for(auto* domain : domains)
    {
      std::unordered_set<ClassBranch*> up;
      ontology_->class_graph_.getUpPtr(branch, up);
      if(up.find(domain) == up.end())
      {
        branch->mothers_.emplaceBack(domain, 1.0, true);
        domain->childs_.emplace_back(branch, 1.0, true);

        branch->nb_updates_++;
        domain->nb_updates_++;
        nb_update++;
      }
    }
  }

  std::string ReasonerRangeDomain::getName()
  {
    return "reasoner range and domain";
  }

  std::string ReasonerRangeDomain::getDescription()
  {
    return "This is an reasoner to deduce new inheritances based on range and domain of properties.";
  }

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerRangeDomain, ontologenius::ReasonerInterface)