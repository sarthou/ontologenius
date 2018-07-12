#include "ontoloGenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"

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
}

void ObjectPropertyBranch_t::setSteady_disjoint(ObjectPropertyBranch_t* disjoint)
{
  steady_.disjoints_.push_back(disjoint);
  disjoints_.push_back(disjoint);
}

void ObjectPropertyBranch_t::setSteady_properties(Properties_t properties)
{
  steady_.properties_ = properties;
  properties_ = properties;
}

void ObjectPropertyBranch_t::setSteady_inverse(ObjectPropertyBranch_t* inverse)
{
  steady_.inverses_.push_back(inverse);
  inverses_.push_back(inverse);
}

void ObjectPropertyBranch_t::setSteady_domain(ClassBranch_t* domain)
{
  steady_.domains_.push_back(domain);
  domains_.push_back(domain);
}

void ObjectPropertyBranch_t::setSteady_range(ClassBranch_t* range)
{
  steady_.ranges_.push_back(range);
  ranges_.push_back(range);
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
  steady_.childs_.push_back(child);
  childs_.push_back(child);
}

void ObjectPropertyBranch_t::setSteady_mother(ObjectPropertyBranch_t* mother)
{
  steady_.mothers_.push_back(mother);
  mothers_.push_back(mother);
}

void ObjectPropertyBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  steady_.dictionary_[lang].push_back(word);
  dictionary_[lang].push_back(word);
}

void ObjectPropertyBranch_t::setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
{
  steady_.dictionary_ = dictionary;
  dictionary_ = dictionary;
}
