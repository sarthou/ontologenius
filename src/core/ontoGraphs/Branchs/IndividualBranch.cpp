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
  if(std::find(steady_.is_a_.begin(), steady_.is_a_.end(), is_a) == steady_.is_a_.end())
    steady_.is_a_.push_back(is_a);
  if(std::find(is_a_.begin(), is_a_.end(), is_a) == is_a_.end())
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
  if(std::find(steady_.same_as_.begin(), steady_.same_as_.end(), same_as) == steady_.same_as_.end())
    steady_.same_as_.push_back(same_as);
  if(std::find(same_as_.begin(), same_as_.end(), same_as) == same_as_.end())
    same_as_.push_back(same_as);
}

void IndividualBranch_t::setSteady_distinct(IndividualBranch_t* distinct)
{
  if(std::find(steady_.distinct_.begin(), steady_.distinct_.end(), distinct) == steady_.distinct_.end())
    steady_.distinct_.push_back(distinct);
  if(std::find(distinct_.begin(), distinct_.end(), distinct) == distinct_.end())
    distinct_.push_back(distinct);
}

void IndividualBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  if(find(steady_.dictionary_[lang].begin(), steady_.dictionary_[lang].end(), word) == steady_.dictionary_[lang].end())
    steady_.dictionary_[lang].push_back(word);

  if(find(dictionary_[lang].begin(), dictionary_[lang].end(), word) == dictionary_[lang].end())
    dictionary_[lang].push_back(word);
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
        if(find(steady_.dictionary_[it.first].begin(), steady_.dictionary_[it.first].end(), name) == steady_.dictionary_[it.first].end())
          steady_.dictionary_[it.first].push_back(name);
    }

    if(dictionary_.find(it.first) == dictionary_.end())
      dictionary_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        if(find(dictionary_[it.first].begin(), dictionary_[it.first].end(), name) == dictionary_[it.first].end())
          dictionary_[it.first].push_back(name);
    }
  }
}

} // namespace ontologenius
