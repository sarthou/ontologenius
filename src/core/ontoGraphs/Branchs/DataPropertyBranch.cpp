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
}

void DataPropertyBranch_t::setSteady_disjoint(DataPropertyBranch_t* disjoint)
{
  if(std::find(steady_.disjoints_.begin(), steady_.disjoints_.end(), disjoint) == steady_.disjoints_.end())
    steady_.disjoints_.push_back(disjoint);
  if(std::find(disjoints_.begin(), disjoints_.end(), disjoint) == disjoints_.end())
    disjoints_.push_back(disjoint);
}

void DataPropertyBranch_t::setSteady_properties(Properties_t properties)
{
  steady_.properties_ = properties;
  properties_ = properties;
}

void DataPropertyBranch_t::setSteady_domain(ClassBranch_t* domain)
{
  if(std::find(steady_.domains_.begin(), steady_.domains_.end(), domain) == steady_.domains_.end())
    steady_.domains_.push_back(domain);
  if(std::find(domains_.begin(), domains_.end(), domain) == domains_.end())
    domains_.push_back(domain);
}

void DataPropertyBranch_t::setSteady_range(std::string range)
{
  data_t tmp;
  tmp.type_ = range;
  steady_.ranges_.push_back(tmp);
  ranges_.push_back(tmp);
}

void DataPropertyBranch_t::setSteady_child(DataPropertyBranch_t* child)
{
  if(std::find(steady_.childs_.begin(), steady_.childs_.end(), child) == steady_.childs_.end())
    steady_.childs_.push_back(child);
  if(std::find(childs_.begin(), childs_.end(), child) == childs_.end())
    childs_.push_back(child);
}

void DataPropertyBranch_t::setSteady_mother(DataPropertyBranch_t* mother)
{
  if(std::find(steady_.mothers_.begin(), steady_.mothers_.end(), mother) == steady_.mothers_.end())
    steady_.mothers_.push_back(mother);
  if(std::find(mothers_.begin(), mothers_.end(), mother) == mothers_.end())
    mothers_.push_back(mother);
}

void DataPropertyBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  if(find(steady_.dictionary_[lang].begin(), steady_.dictionary_[lang].end(), word) == steady_.dictionary_[lang].end())
    steady_.dictionary_[lang].push_back(word);

  if(find(dictionary_[lang].begin(), dictionary_[lang].end(), word) == dictionary_[lang].end())
    dictionary_[lang].push_back(word);
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
