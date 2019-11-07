#include "ontoloGenius/core/reasoner/plugins/ReasonerRangeDomain.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerRangeDomain::postReason()
{
  postReasonIndividuals();
  postReasonClasses();
}

void ReasonerRangeDomain::postReasonIndividuals()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();

  std::map<std::string, std::vector<std::string>>::iterator it_range;
  std::map<std::string, std::vector<std::string>>::iterator it_domain;

  for(size_t indiv_i = 0; indiv_i < indiv_size; indiv_i++)
    if(indiv[indiv_i]->updated_ == true)
    {
      it_range = indiv[indiv_i]->flags_.find("range");
      if(it_range != indiv[indiv_i]->flags_.end())
      {
        for(std::string prop : it_range->second)
          deduceRange(indiv[indiv_i], prop);
        indiv[indiv_i]->flags_.erase("range");
      }

      it_domain = indiv[indiv_i]->flags_.find("domain");
      if(it_domain != indiv[indiv_i]->flags_.end())
      {
        for(std::string prop : it_domain->second)
          deduceDomain(indiv[indiv_i], prop);
        indiv[indiv_i]->flags_.erase("domain");
      }
    }
}

void ReasonerRangeDomain::deduceRange(IndividualBranch_t* branch, std::string& prop)
{
  for(size_t i = 0; i < branch->object_relations_.size(); i++)
    if(branch->object_relations_[i].first->value() == prop)
      deduceObjRange(branch, i);
}

void ReasonerRangeDomain::deduceDomain(IndividualBranch_t* branch, std::string& prop)
{
  for(size_t i = 0; i < branch->object_relations_.size(); i++)
    if(branch->object_relations_[i].first->value() == prop)
      deduceObjDomain(branch, i);

  for(size_t i = 0; i < branch->data_relations_.size(); i++)
    if(branch->data_relations_[i].first->value() == prop)
      deduceDatDomain(branch, i);
}

void ReasonerRangeDomain::deduceObjRange(IndividualBranch_t* branch, size_t index)
{
  std::unordered_set<ClassBranch_t*> ranges;
  std::unordered_set<ObjectPropertyBranch_t*> props;
  props.insert(branch->object_relations_[index].first);
  while(ranges.size() == 0)
  {
    for(auto prop : props)
      ontology_->object_property_graph_.getRangePtr(prop, ranges, 0);

    if(ranges.size() == 0)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      for(auto prop : props)
      {
        ontology_->object_property_graph_.getUpPtr(prop, prop_up, 1);
        prop_up.erase(prop);
      }

      if(prop_up.size() == 0)
        break;
      else
        props = prop_up;
    }
  }

  for(auto range : ranges)
  {
    std::unordered_set<ClassBranch_t*> up;
    ontology_->individual_graph_.getUpPtr(branch->object_relations_[index].second, up);
    if(up.find(range) == up.end())
    {
      branch->object_relations_[index].second->is_a_.push_back(range);
      range->individual_childs_.push_back(branch->object_relations_[index].second);

      branch->object_relations_[index].second->updated_ = true;
      range->updated_ = true;
    }
  }
}

void ReasonerRangeDomain::deduceObjDomain(IndividualBranch_t* branch, size_t index)
{
  std::unordered_set<ClassBranch_t*> domains;
  std::unordered_set<ObjectPropertyBranch_t*> props;
  props.insert(branch->object_relations_[index].first);
  while(domains.size() == 0)
  {
    for(auto prop : props)
      ontology_->object_property_graph_.getDomainPtr(prop, domains, 0);

    if(domains.size() == 0)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      for(auto prop : props)
      {
        ontology_->object_property_graph_.getUpPtr(prop, prop_up, 1);
        prop_up.erase(prop);
      }

      if(prop_up.size() == 0)
        break;
      else
        props = prop_up;
    }
  }

  for(auto domain : domains)
  {
    std::unordered_set<ClassBranch_t*> up;
    ontology_->individual_graph_.getUpPtr(branch, up);
    if(up.find(domain) == up.end())
    {
      branch->is_a_.push_back(domain);
      domain->individual_childs_.push_back(branch);

      branch->updated_ = true;
      domain->updated_ = true;
    }
  }
}

void ReasonerRangeDomain::deduceDatDomain(IndividualBranch_t* branch, size_t index)
{
  std::unordered_set<ClassBranch_t*> domains;
  std::unordered_set<DataPropertyBranch_t*> props;
  props.insert(branch->data_relations_[index].first);
  while(domains.size() == 0)
  {
    for(auto prop : props)
      ontology_->data_property_graph_.getDomainPtr(prop, domains, 0);

    if(domains.size() == 0)
    {
      std::unordered_set<DataPropertyBranch_t*> prop_up;
      for(auto prop : props)
      {
        ontology_->data_property_graph_.getUpPtr(prop, prop_up, 1);
        prop_up.erase(prop);
      }

      if(prop_up.size() == 0)
        break;
      else
        props = prop_up;
    }
  }

  for(auto domain : domains)
  {
    std::unordered_set<ClassBranch_t*> up;
    ontology_->individual_graph_.getUpPtr(branch, up);
    if(up.find(domain) == up.end())
    {
      branch->is_a_.push_back(domain);
      domain->individual_childs_.push_back(branch);

      branch->updated_ = true;
      domain->updated_ = true;
    }
  }
}


void ReasonerRangeDomain::postReasonClasses()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->class_graph_.mutex_);
  std::vector<ClassBranch_t*> classes = ontology_->class_graph_.get();
  size_t classes_size = classes.size();

  std::map<std::string, std::vector<std::string>>::iterator it_range;
  std::map<std::string, std::vector<std::string>>::iterator it_domain;

  for(size_t class_i = 0; class_i < classes_size; class_i++)
    if(classes[class_i]->updated_ == true)
    {
      it_range = classes[class_i]->flags_.find("range");
      if(it_range != classes[class_i]->flags_.end())
      {
        for(std::string prop : it_range->second)
          deduceRange(classes[class_i], prop);
        classes[class_i]->flags_.erase("range");
      }

      it_domain = classes[class_i]->flags_.find("domain");
      if(it_domain != classes[class_i]->flags_.end())
      {
        for(std::string prop : it_domain->second)
          deduceDomain(classes[class_i], prop);
        classes[class_i]->flags_.erase("domain");
      }
    }
}

void ReasonerRangeDomain::deduceRange(ClassBranch_t* branch, std::string& prop)
{
  for(size_t i = 0; i < branch->object_relations_.size(); i++)
    if(branch->object_relations_[i].first->value() == prop)
      deduceObjRange(branch, i);
}

void ReasonerRangeDomain::deduceDomain(ClassBranch_t* branch, std::string& prop)
{
  for(size_t i = 0; i < branch->object_relations_.size(); i++)
    if(branch->object_relations_[i].first->value() == prop)
      deduceObjDomain(branch, i);

  for(size_t i = 0; i < branch->data_relations_.size(); i++)
    if(branch->data_relations_[i].first->value() == prop)
      deduceDatDomain(branch, i);
}

void ReasonerRangeDomain::deduceObjRange(ClassBranch_t* branch, size_t index)
{
  std::unordered_set<ClassBranch_t*> ranges;
  std::unordered_set<ObjectPropertyBranch_t*> props;
  props.insert(branch->object_relations_[index].first);
  while(ranges.size() == 0)
  {
    for(auto prop : props)
      ontology_->object_property_graph_.getRangePtr(prop, ranges, 0);

    if(ranges.size() == 0)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      for(auto prop : props)
      {
        ontology_->object_property_graph_.getUpPtr(prop, prop_up, 1);
        prop_up.erase(prop);
      }

      if(prop_up.size() == 0)
        break;
      else
        props = prop_up;
    }
  }

  for(auto range : ranges)
  {
    std::unordered_set<ClassBranch_t*> up;
    ontology_->class_graph_.getUpPtr(branch->object_relations_[index].second, up);
    if(up.find(range) == up.end())
    {
      branch->object_relations_[index].second->mothers_.push_back(range);
      range->childs_.push_back(branch->object_relations_[index].second);

      branch->object_relations_[index].second->updated_ = true;
      range->updated_ = true;
    }
  }
}

void ReasonerRangeDomain::deduceObjDomain(ClassBranch_t* branch, size_t index)
{
  std::unordered_set<ClassBranch_t*> domains;
  std::unordered_set<ObjectPropertyBranch_t*> props;
  props.insert(branch->object_relations_[index].first);
  while(domains.size() == 0)
  {
    for(auto prop : props)
      ontology_->object_property_graph_.getDomainPtr(prop, domains, 0);

    if(domains.size() == 0)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      for(auto prop : props)
      {
        ontology_->object_property_graph_.getUpPtr(prop, prop_up, 1);
        prop_up.erase(prop);
      }

      if(prop_up.size() == 0)
        break;
      else
        props = prop_up;
    }
  }

  for(auto domain : domains)
  {
    std::unordered_set<ClassBranch_t*> up;
    ontology_->class_graph_.getUpPtr(branch, up);
    if(up.find(domain) == up.end())
    {
      branch->mothers_.push_back(domain);
      domain->childs_.push_back(branch);

      branch->updated_ = true;
      domain->updated_ = true;
    }
  }
}

void ReasonerRangeDomain::deduceDatDomain(ClassBranch_t* branch, size_t index)
{
  std::unordered_set<ClassBranch_t*> domains;
  std::unordered_set<DataPropertyBranch_t*> props;
  props.insert(branch->data_relations_[index].first);
  while(domains.size() == 0)
  {
    for(auto prop : props)
      ontology_->data_property_graph_.getDomainPtr(prop, domains, 0);

    if(domains.size() == 0)
    {
      std::unordered_set<DataPropertyBranch_t*> prop_up;
      for(auto prop : props)
      {
        ontology_->data_property_graph_.getUpPtr(prop, prop_up, 1);
        prop_up.erase(prop);
      }

      if(prop_up.size() == 0)
        break;
      else
        props = prop_up;
    }
  }

  for(auto domain : domains)
  {
    std::unordered_set<ClassBranch_t*> up;
    ontology_->class_graph_.getUpPtr(branch, up);
    if(up.find(domain) == up.end())
    {
      branch->mothers_.push_back(domain);
      domain->childs_.push_back(branch);

      branch->updated_ = true;
      domain->updated_ = true;
    }
  }
}

std::string ReasonerRangeDomain::getName()
{
  return "reasoner range and domain";
}

std::string ReasonerRangeDomain::getDesciption()
{
  return "This is an reasoner to deduce new inheritances based on range and domain of properties.";
}

PLUGINLIB_EXPORT_CLASS(ReasonerRangeDomain, ReasonerInterface)

} // namespace ontologenius
