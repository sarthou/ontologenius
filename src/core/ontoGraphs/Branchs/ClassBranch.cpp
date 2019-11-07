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

  steady_.object_relations_.clear();
  for(size_t i = 0; i < object_relations_.size(); i++)
    steady_.object_relations_.push_back(object_relations_[i]);

  steady_.data_relations_.clear();
  for(size_t i = 0; i < data_relations_.size(); i++)
    steady_.data_relations_.push_back(data_relations_[i]);
}

void ClassBranch_t::setSteady_disjoint(ClassBranch_t* disjoint)
{
  conditionalPushBack(steady_.disjoints_, disjoint);
  conditionalPushBack(disjoints_, disjoint);
}

void ClassBranch_t::setSteady_child(const ClassElement_t& child)
{
  conditionalPushBack(steady_.childs_, child);
  conditionalPushBack(childs_, child);
}

void ClassBranch_t::setSteady_individual_child(const IndividualElement_t& child)
{
  conditionalPushBack(steady_.individual_childs_, child);
  conditionalPushBack(individual_childs_, child);
}

void ClassBranch_t::setSteady_mother(const ClassElement_t& mother)
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

void ClassBranch_t::setSteady_objectRelation(const ClassObjectRelationElement_t& object_relation)
{
  steady_.object_relations_.push_back(object_relation);
  object_relations_.push_back(object_relation);
}

void ClassBranch_t::setSteady_dataRelation(const ClassDataRelationElement_t& data_relation)
{
  steady_.data_relations_.push_back(data_relation);
  data_relations_.push_back(data_relation);
}

} // namespace ontologenius
