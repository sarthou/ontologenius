#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"

#include <algorithm>

namespace ontologenius {

void ClassBranch_t::setFullSteady()
{
  steady_.disjoints_.clear();
  for(size_t i = 0; i < disjoints_.size(); i++)
    steady_.disjoints_.push_back(disjoints_[i]);

  steady_.childs_.clear();
  for(size_t i = 0; i < childs_.size(); i++)
    steady_.childs_.push_back(childs_[i]);

  steady_.individual_childs_.clear();
  for(size_t i = 0; i < individual_childs_.size(); i++)
    steady_.individual_childs_.push_back(individual_childs_[i]);

  steady_.mothers_.clear();
  for(size_t i = 0; i < mothers_.size(); i++)
    steady_.mothers_.push_back(mothers_[i]);

  steady_.dictionary_.clear();
  for(auto& it : dictionary_)
  {
    std::vector<std::string> tmp;
    for(size_t i = 0; i < it.second.size(); i++)
      tmp.push_back(it.second[i]);
    steady_.dictionary_[it.first] = tmp;
  }

  steady_.muted_dictionary_.clear();
  for(auto& it : muted_dictionary_)
  {
    std::vector<std::string> tmp;
    for(size_t i = 0; i < it.second.size(); i++)
      tmp.push_back(it.second[i]);
    steady_.muted_dictionary_[it.first] = tmp;
  }

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
}

void ClassBranch_t::setSteady_disjoint(ClassBranch_t* disjoint)
{
  conditionalPushBack(steady_.disjoints_, disjoint);
  conditionalPushBack(disjoints_, disjoint);
}

void ClassBranch_t::setSteady_child(const Single_t<ClassBranch_t*>& child)
{
  conditionalPushBack(steady_.childs_, child);
  conditionalPushBack(childs_, child);
}

void ClassBranch_t::setSteady_individual_child(const Single_t<IndividualBranch_t*>& child)
{
  conditionalPushBack(steady_.individual_childs_, child);
  conditionalPushBack(individual_childs_, child);
}

void ClassBranch_t::setSteady_mother(const Single_t<ClassBranch_t*>& mother)
{
  conditionalPushBack(steady_.mothers_, mother);
  conditionalPushBack(mothers_, mother);
}

void ClassBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(steady_.dictionary_[lang], word);
  conditionalPushBack(dictionary_[lang], word);
}

void ClassBranch_t::setSteady_muted_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(steady_.muted_dictionary_[lang], word);
  conditionalPushBack(muted_dictionary_[lang], word);
}

void ClassBranch_t::setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
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

void ClassBranch_t::setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
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

void ClassBranch_t::setSteady_object_properties_name(ObjectPropertyBranch_t* object_properties_name)
{
  steady_.object_properties_name_.push_back(object_properties_name);
  object_properties_name_.push_back(object_properties_name);
}

void ClassBranch_t::setSteady_object_properties_on(ClassBranch_t* object_properties_on)
{
  steady_.object_properties_on_.push_back(object_properties_on);
  object_properties_on_.push_back(object_properties_on);
}

void ClassBranch_t::setSteady_data_properties_name(DataPropertyBranch_t* data_properties_name)
{
  steady_.data_properties_name_.push_back(data_properties_name);
  data_properties_name_.push_back(data_properties_name);
}

void ClassBranch_t::setSteady_data_properties_data(data_t data_properties_data)
{
  steady_.data_properties_data_.push_back(data_properties_data);
  data_properties_data_.push_back(data_properties_data);
}

} // namespace ontologenius
