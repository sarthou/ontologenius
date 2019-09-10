#include "ontoloGenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"

#include <algorithm>

namespace ontologenius {

void ObjectPropertyBranch_t::setFullSteady()
{
  steady_.disjoints_.clear();
  for(size_t i = 0; i < disjoints_.size(); i++)
    steady_.disjoints_.push_back(disjoints_[i]);

  steady_.properties_ = properties_;

  steady_.inverses_.clear();
  for(size_t i = 0; i < inverses_.size(); i++)
    steady_.inverses_.push_back(inverses_[i]);

  steady_.domains_.clear();
  for(size_t i = 0; i < domains_.size(); i++)
    steady_.domains_.push_back(domains_[i]);

  steady_.ranges_.clear();
  for(size_t i = 0; i < ranges_.size(); i++)
    steady_.ranges_.push_back(ranges_[i]);

  steady_.chains_.clear();
  for(size_t i = 0; i < chains_.size(); i++)
  {
    std::vector<ObjectPropertyBranch_t*> tmp;
    for(size_t j = 0; j < chains_[i].size(); j++)
      tmp.push_back(chains_[i][j]);
    steady_.chains_.push_back(tmp);
  }

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

void ObjectPropertyBranch_t::setSteady_disjoint(ObjectPropertyBranch_t* disjoint)
{
  conditionalPushBack(steady_.disjoints_, disjoint);
  conditionalPushBack(disjoints_, disjoint);
}

void ObjectPropertyBranch_t::setSteady_properties(Properties_t properties)
{
  steady_.properties_ = properties;
  properties_ = properties;
}

void ObjectPropertyBranch_t::setSteady_inverse(ObjectPropertyBranch_t* inverse)
{
  conditionalPushBack(steady_.inverses_, inverse);
  conditionalPushBack(inverses_, inverse);
}

void ObjectPropertyBranch_t::setSteady_domain(ClassBranch_t* domain)
{
  conditionalPushBack(steady_.domains_, domain);
  conditionalPushBack(domains_, domain);
}

void ObjectPropertyBranch_t::setSteady_range(ClassBranch_t* range)
{
  conditionalPushBack(steady_.ranges_, range);
  conditionalPushBack(ranges_, range);
}

void ObjectPropertyBranch_t::set_chain(std::vector<ObjectPropertyBranch_t*> chain)
{
  chains_.push_back(chain);
}

void ObjectPropertyBranch_t::setSteady_chain(std::vector<std::string> chain)
{
  steady_.str_chains_.push_back(chain);
}

void ObjectPropertyBranch_t::setSteady_child(ObjectPropertyBranch_t* child)
{
  conditionalPushBack(steady_.childs_, child);
  conditionalPushBack(childs_, child);
}

void ObjectPropertyBranch_t::setSteady_mother(ObjectPropertyBranch_t* mother)
{
  conditionalPushBack(steady_.mothers_, mother);
  conditionalPushBack(mothers_, mother);
}

void ObjectPropertyBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(steady_.dictionary_[lang], word);
  conditionalPushBack(dictionary_[lang], word);
}

void ObjectPropertyBranch_t::setSteady_muted_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(steady_.muted_dictionary_[lang], word);
  conditionalPushBack(muted_dictionary_[lang], word);
}

void ObjectPropertyBranch_t::setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
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

void ObjectPropertyBranch_t::setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
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
