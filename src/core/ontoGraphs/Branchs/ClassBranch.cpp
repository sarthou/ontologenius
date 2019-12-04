#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"

#include <algorithm>

namespace ontologenius {

void ClassBranch_t::setSteady_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(dictionary_.spoken_[lang], word);
  conditionalPushBack(steady_dictionary_.spoken_[lang], word);
}

void ClassBranch_t::setSteady_muted_dictionary(std::string lang, std::string word)
{
  conditionalPushBack(dictionary_.muted_[lang], word);
  conditionalPushBack(steady_dictionary_.muted_[lang], word);
}

void ClassBranch_t::setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
{
  for(auto it : dictionary)
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

void ClassBranch_t::setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary)
{
  for(auto it : dictionary)
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

} // namespace ontologenius
