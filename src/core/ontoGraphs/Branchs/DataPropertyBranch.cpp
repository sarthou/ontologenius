#include "ontoloGenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

#include <algorithm>

namespace ontologenius {

void DataPropertyBranch_t::setFullSteady()
{
  steady_.disjoints_.clear();
  for(size_t i = 0; i < disjoints_.size(); i++)
    steady_.disjoints_.push_back(disjoints_[i]);

  steady_.properties_ = properties_;

  steady_.domains_.clear();
  for(size_t i = 0; i < domains_.size(); i++)
    steady_.domains_.push_back(domains_[i]);

  steady_.ranges_.clear();
  for(size_t i = 0; i < ranges_.size(); i++)
    steady_.ranges_.push_back(ranges_[i]);

  steady_.childs_.clear();
  for(size_t i = 0; i < childs_.size(); i++)
    steady_.childs_.push_back(childs_[i]);

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
}

void DataPropertyBranch_t::setSteady_disjoint(DataPropertyBranch_t* disjoint)
{
  conditionalPushBack(steady_.disjoints_, disjoint);
  conditionalPushBack(disjoints_, disjoint);
}

void DataPropertyBranch_t::setSteady_properties(Properties_t properties)
{
  steady_.properties_ = properties;
  properties_ = properties;
}

void DataPropertyBranch_t::setSteady_domain(const ClassElement_t& domain)
{
  conditionalPushBack(steady_.domains_, domain);
  conditionalPushBack(domains_, domain);
}

void DataPropertyBranch_t::setSteady_range(std::string range)
{
  data_t tmp;
  tmp.type_ = range;
  steady_.ranges_.push_back(tmp);
  ranges_.push_back(tmp);
}

void DataPropertyBranch_t::setSteady_child(const DataPropertyElement_t& child)
{
  conditionalPushBack(steady_.childs_, child);
  conditionalPushBack(childs_, child);
}

void DataPropertyBranch_t::setSteady_mother(const DataPropertyElement_t& mother)
{
  conditionalPushBack(steady_.mothers_, mother);
  conditionalPushBack(mothers_, mother);
}

void DataPropertyBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(steady_.dictionary_[lang], word);
  conditionalPushBack(dictionary_[lang], word);
}

void DataPropertyBranch_t::setSteady_muted_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(steady_.muted_dictionary_[lang], word);
  conditionalPushBack(muted_dictionary_[lang], word);
}

void DataPropertyBranch_t::setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
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

void DataPropertyBranch_t::setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
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

} // namespace ontologenius
