#include "ontoloGenius/arguer/plugins/ArguerDictionary.h"
#include <pluginlib/class_list_macros.h>

void ArguerDictionary::preReason()
{

}

void ArguerDictionary::postReason()
{
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  for(size_t indiv_i = 0; indiv_i < indiv.size(); indiv_i++)
    if(indiv[indiv_i]->updated_ == true)
    {
      split(indiv[indiv_i]);
      createLowerCase(indiv[indiv_i]);
      replaceQuote(indiv[indiv_i]);
    }
}

void ArguerDictionary::split(IndividualBranch_t* indiv)
{
  std::map<std::string, std::vector<std::string>>::iterator it;
  for (it = indiv->dictionary_.begin(); it != indiv->dictionary_.end(); ++it)
  {
    for(size_t i = 0; i < it->second.size(); i++)
    {
      std::string tmp = it->second[i];
      std::replace( tmp.begin(), tmp.end(), '_', ' ');
      if (std::find(it->second.begin(), it->second.end(), tmp) == it->second.end())
        it->second.push_back(tmp);
    }

    for(size_t i = 0; i < it->second.size(); i++)
    {
      std::string tmp = "";
      tmp = tmp + it->second[i][0];
      for(size_t char_i = 1; char_i < it->second[i].size(); char_i++)
      {
        if((it->second[i][char_i] >= 'A') && (it->second[i][char_i] <= 'Z'))
          if(it->second[i][char_i - 1] != ' ')
            tmp = tmp + ' ';
        tmp = tmp + it->second[i][char_i];
      }
      if (std::find(it->second.begin(), it->second.end(), tmp) == it->second.end())
        it->second.push_back(tmp);
    }
  }
}

void ArguerDictionary::createLowerCase(IndividualBranch_t* indiv)
{
  std::map<std::string, std::vector<std::string>>::iterator it;
  for (it = indiv->dictionary_.begin(); it != indiv->dictionary_.end(); ++it)
  {
    for(size_t i = 0; i < it->second.size(); i++)
    {
      std::string tmp = "";
      tmp.resize(it->second[i].size());
      std::transform(it->second[i].begin(), it->second[i].end(), tmp.begin(), ::tolower);
      if (std::find(it->second.begin(), it->second.end(), tmp) == it->second.end())
        it->second.push_back(tmp);
    }
  }
}

void ArguerDictionary::replaceQuote(IndividualBranch_t* indiv)
{
  std::map<std::string, std::vector<std::string>>::iterator it;
  for (it = indiv->dictionary_.begin(); it != indiv->dictionary_.end(); ++it)
  {
    for(size_t i = 0; i < it->second.size(); i++)
    {
      std::string tmp = it->second[i];
      tmp.erase(std::remove(tmp.begin(), tmp.end(), '\''), tmp.end());
      if (std::find(it->second.begin(), it->second.end(), tmp) == it->second.end())
        it->second.push_back(tmp);
    }
  }
}

std::string ArguerDictionary::getName()
{
  return "arguer dictionary";
}

std::string ArguerDictionary::getDesciption()
{
  return "This arguer creates several alternative dictionaries to avoid too many restrictive labels.";
}

PLUGINLIB_EXPORT_CLASS(ArguerDictionary, ArguerInterface)
