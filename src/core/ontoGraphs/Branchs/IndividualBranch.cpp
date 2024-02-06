#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"

#include <algorithm>

namespace ontologenius {

void IndividualBranch_t::setSteady_dictionary(const std::string& lang, const std::string& word)
{
  conditionalPushBack(dictionary_.spoken_[lang], word);
  conditionalPushBack(steady_dictionary_.spoken_[lang], word);
}

void IndividualBranch_t::setSteady_muted_dictionary(const std::string& lang, const std::string& word)
{
  conditionalPushBack(dictionary_.muted_[lang], word);
  conditionalPushBack(steady_dictionary_.muted_[lang], word);
}

void IndividualBranch_t::setSteady_dictionary(const std::map<std::string, std::vector<std::string>>& dictionary)
{
  for(const auto& it : dictionary)
  {
    if(dictionary_.spoken_.find(it.first) == dictionary_.spoken_.end())
      dictionary_.spoken_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        conditionalPushBack(dictionary_.spoken_[it.first], name);
    }

    if(steady_dictionary_.spoken_.find(it.first) == steady_dictionary_.spoken_.end())
      steady_dictionary_.spoken_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        conditionalPushBack(steady_dictionary_.spoken_[it.first], name);
    }
  }
}

void IndividualBranch_t::setSteady_muted_dictionary(const std::map<std::string, std::vector<std::string>>& dictionary)
{
  for(const auto& it : dictionary)
  {
    if(dictionary_.muted_.find(it.first) == dictionary_.muted_.end())
      dictionary_.muted_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        conditionalPushBack(dictionary_.muted_[it.first], name);
    }

    if(steady_dictionary_.muted_.find(it.first) == steady_dictionary_.muted_.end())
      steady_dictionary_.muted_[it.first] = it.second;
    else
    {
      for(const auto& name : it.second)
        conditionalPushBack(steady_dictionary_.muted_[it.first], name);
    }
  }
}

int IndividualBranch_t::objectRelationExists(ObjectPropertyBranch_t* property, IndividualBranch_t* individual)
{
  for(size_t i = 0; i < object_relations_.size(); i++)
  {
    if(object_relations_[i].first == property)
      if(object_relations_[i].second == individual)
        return i;
  }
  return -1;
}

int IndividualBranch_t::dataRelationExists(DataPropertyBranch_t* property, LiteralNode* data)
{
  for(size_t i = 0; i < data_relations_.size(); i++)
  {
    if(data_relations_[i].first == property)
      if(data_relations_[i].second == data)
        return i;
  }
  return -1;
}

} // namespace ontologenius
