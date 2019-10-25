#include "ontoloGenius/core/ontoGraphs/Branchs/IndividualBranch.h"

#include <algorithm>

namespace ontologenius {

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
  conditionalPushBack(steady_.is_a_, is_a);
  conditionalPushBack(is_a_, is_a);
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
  conditionalPushBack(steady_.same_as_, same_as);
  conditionalPushBack(same_as_, same_as);
}

void IndividualBranch_t::setSteady_distinct(IndividualBranch_t* distinct)
{
  conditionalPushBack(steady_.distinct_, distinct);
  conditionalPushBack(distinct_, distinct);
}

void IndividualBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(steady_.dictionary_[lang], word);
  conditionalPushBack(dictionary_[lang], word);
}

void IndividualBranch_t::setSteady_muted_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(steady_.muted_dictionary_[lang], word);
  conditionalPushBack(muted_dictionary_[lang], word);
}

void IndividualBranch_t::setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
{
  for(auto it : dictionary)
  {
    if(steady_.dictionary_.find(it.first) == steady_.dictionary_.end())
      steady_.dictionary_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        conditionalPushBack(steady_.dictionary_[it.first], name);
    }

    if(dictionary_.find(it.first) == dictionary_.end())
      dictionary_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        conditionalPushBack(dictionary_[it.first], name);
    }
  }
}

void IndividualBranch_t::setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
{
  for(auto it : dictionary)
  {
    if(steady_.muted_dictionary_.find(it.first) == steady_.muted_dictionary_.end())
      steady_.muted_dictionary_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        conditionalPushBack(steady_.muted_dictionary_[it.first], name);
    }

    if(muted_dictionary_.find(it.first) == muted_dictionary_.end())
      muted_dictionary_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        conditionalPushBack(muted_dictionary_[it.first], name);
    }
  }
}

int IndividualBranch_t::ObjectPropertyExistSteady(ObjectPropertyBranch_t* property, IndividualBranch_t* individual)
{
  int res = -1;
  for(size_t i = 0; i < steady_.object_properties_name_.size(); i++)
  {
    if(steady_.object_properties_name_[i] == property)
      if(steady_.object_properties_on_[i] == individual)
      {
        res = i;
        break;
      }
  }

  return res;
}

int IndividualBranch_t::ObjectPropertyExist(ObjectPropertyBranch_t* property, IndividualBranch_t* individual)
{
  int res = -1;
  for(size_t i = 0; i < object_properties_name_.size(); i++)
  {
    if(object_properties_name_[i] == property)
      if(object_properties_on_[i] == individual)
      {
        res = i;
        break;
      }
  }

  return res;
}

} // namespace ontologenius
