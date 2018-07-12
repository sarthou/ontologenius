#include "ontoloGenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

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
  steady_.disjoints_.push_back(disjoint);
  disjoints_.push_back(disjoint);
}

void DataPropertyBranch_t::setSteady_properties(Properties_t properties)
{
  steady_.properties_ = properties;
  properties_ = properties;
}

void DataPropertyBranch_t::setSteady_domain(ClassBranch_t* domain)
{
  steady_.domains_.push_back(domain);
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
  steady_.childs_.push_back(child);
  childs_.push_back(child);
}

void DataPropertyBranch_t::setSteady_mother(DataPropertyBranch_t* mother)
{
  steady_.mothers_.push_back(mother);
  mothers_.push_back(mother);
}

void DataPropertyBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  steady_.dictionary_[lang].push_back(word);
  dictionary_[lang].push_back(word);
}

void DataPropertyBranch_t::setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
{
  steady_.dictionary_ = dictionary;
  dictionary_ = dictionary;
}
