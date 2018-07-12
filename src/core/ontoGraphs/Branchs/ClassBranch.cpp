#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"

void ClassBranch_t::setFullSteady()
{
  steady_.disjoints_.clear();
  for(size_t i = 0; i < disjoints_.size(); i++)
    steady_.disjoints_.push_back(disjoints_[i]);

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

void ClassBranch_t::setSteady_disjoint(ClassBranch_t* disjoint)
{
  steady_.disjoints_.push_back(disjoint);
  disjoints_.push_back(disjoint);
}

void ClassBranch_t::setSteady_child(ClassBranch_t* child)
{
  steady_.childs_.push_back(child);
  childs_.push_back(child);
}

void ClassBranch_t::setSteady_mother(ClassBranch_t* mother)
{
  steady_.mothers_.push_back(mother);
  mothers_.push_back(mother);
}

void ClassBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  steady_.dictionary_[lang].push_back(word);
  dictionary_[lang].push_back(word);
}

void ClassBranch_t::setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
{
  steady_.dictionary_ = dictionary;
  dictionary_ = dictionary;
}
