#include "ontoloGenius/core/ontoGraphs/Branchs/IndividualBranch.h"

void IndividualBranch_t::setFullSteady()
{
  steady_.is_a_.clear();
  for(size_t i = 0; i < is_a_.size(); i++)
    steady_.is_a_.push_back(is_a_[i]);

  steady_.object_properties_name_.clear();
  for(size_t i = 0; i < object_properties_name_.size(); i++)
    steady_.object_properties_name_.push_back(object_properties_name_[i]);

  steady_.object_properties_on_.clear();
  for(size_t i = 0; i < object_properties_on_.size(); i++)
    steady_.object_properties_on_.push_back(object_properties_on_[i]);

  steady_.data_properties_name_.clear();
  for(size_t i = 0; i < data_properties_name_.size(); i++)
    steady_.data_properties_name_.push_back(data_properties_name_[i]);

  steady_.data_properties_data_.clear();
  for(size_t i = 0; i < data_properties_data_.size(); i++)
    steady_.data_properties_data_.push_back(data_properties_data_[i]);

  steady_.same_as_.clear();
  for(size_t i = 0; i < same_as_.size(); i++)
    steady_.same_as_.push_back(same_as_[i]);

  steady_.distinct_.clear();
  for(size_t i = 0; i < distinct_.size(); i++)
    steady_.distinct_.push_back(distinct_[i]);
}

void IndividualBranch_t::setSteady_is_a(ClassBranch_t* is_a)
{
  steady_.is_a_.push_back(is_a);
  is_a_.push_back(is_a);
}

void IndividualBranch_t::setSteady_object_properties_name(ObjectPropertyBranch_t* object_properties_name)
{
  steady_.object_properties_name_.push_back(object_properties_name);
  object_properties_name_.push_back(object_properties_name);
}

void IndividualBranch_t::setSteady_object_properties_on(IndividualBranch_t* object_properties_on)
{
  steady_.object_properties_on_.push_back(object_properties_on);
  object_properties_on_.push_back(object_properties_on);
}

void IndividualBranch_t::setSteady_data_properties_name(DataPropertyBranch_t* data_properties_name)
{
  steady_.data_properties_name_.push_back(data_properties_name);
  data_properties_name_.push_back(data_properties_name);
}

void IndividualBranch_t::setSteady_data_properties_data(data_t data_properties_data)
{
  steady_.data_properties_data_.push_back(data_properties_data);
  data_properties_data_.push_back(data_properties_data);
}

void IndividualBranch_t::setSteady_same_as(IndividualBranch_t* same_as)
{
  steady_.same_as_.push_back(same_as);
  same_as_.push_back(same_as);
}

void IndividualBranch_t::setSteady_distinct(IndividualBranch_t* distinct)
{
  steady_.distinct_.push_back(distinct);
  distinct_.push_back(distinct);
}
