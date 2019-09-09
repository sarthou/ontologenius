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
  if(std::find(steady_.disjoints_.begin(), steady_.disjoints_.end(), disjoint) == steady_.disjoints_.end())
    steady_.disjoints_.push_back(disjoint);
  if(std::find(disjoints_.begin(), disjoints_.end(), disjoint) == disjoints_.end())
    disjoints_.push_back(disjoint);
}

void ClassBranch_t::setSteady_child(ClassBranch_t* child)
{
  if(std::find(steady_.childs_.begin(), steady_.childs_.end(), child) == steady_.childs_.end())
    steady_.childs_.push_back(child);
  if(std::find(childs_.begin(), childs_.end(), child) == childs_.end())
    childs_.push_back(child);
}

void ClassBranch_t::setSteady_individual_child(IndividualBranch_t* child)
{
  if(std::find(steady_.individual_childs_.begin(), steady_.individual_childs_.end(), child) == steady_.individual_childs_.end())
    steady_.individual_childs_.push_back(child);
  if(std::find(individual_childs_.begin(), individual_childs_.end(), child) == individual_childs_.end())
    individual_childs_.push_back(child);
}

void ClassBranch_t::setSteady_mother(ClassBranch_t* mother)
{
  if(std::find(steady_.mothers_.begin(), steady_.mothers_.end(), mother) == steady_.mothers_.end())
    steady_.mothers_.push_back(mother);
  if(std::find(mothers_.begin(), mothers_.end(), mother) == mothers_.end())
    mothers_.push_back(mother);
}

void ClassBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  if(find(steady_.dictionary_[lang].begin(), steady_.dictionary_[lang].end(), word) == steady_.dictionary_[lang].end())
    steady_.dictionary_[lang].push_back(word);

  if(find(dictionary_[lang].begin(), dictionary_[lang].end(), word) == dictionary_[lang].end())
    dictionary_[lang].push_back(word);
}

void ClassBranch_t::setSteady_muted_dictionary(std::string lang, std::string word)
{
  if(find(steady_.muted_dictionary_[lang].begin(), steady_.muted_dictionary_[lang].end(), word) == steady_.muted_dictionary_[lang].end())
    steady_.dictionary_[lang].push_back(word);

  if(find(muted_dictionary_[lang].begin(), muted_dictionary_[lang].end(), word) == muted_dictionary_[lang].end())
    muted_dictionary_[lang].push_back(word);
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

void ClassBranch_t::setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
{
  for(auto it : dictionary)
  {
    if(steady_.muted_dictionary_.find(it.first) == steady_.muted_dictionary_.end())
      steady_.muted_dictionary_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        if(find(steady_.muted_dictionary_[it.first].begin(), steady_.muted_dictionary_[it.first].end(), name) == steady_.muted_dictionary_[it.first].end())
          steady_.muted_dictionary_[it.first].push_back(name);
    }

    if(muted_dictionary_.find(it.first) == muted_dictionary_.end())
      muted_dictionary_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        if(find(muted_dictionary_[it.first].begin(), muted_dictionary_[it.first].end(), name) == muted_dictionary_[it.first].end())
          muted_dictionary_[it.first].push_back(name);
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
