#include "ontologenius/core/reasoner/plugins/ReasonerDictionary.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerDictionary::postReason()
{
  size_t graph_size;
  {
    std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
    std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
    graph_size = indiv.size();
    for(size_t i = 0; i < graph_size; i++)
      updateDictionary(indiv[i]);
  }

  {
    std::vector<ClassBranch_t*> classes = ontology_->class_graph_.getSafe();
    std::lock_guard<std::shared_timed_mutex> lock(ontology_->class_graph_.mutex_);
    graph_size = classes.size();
    for(size_t i = 0; i < graph_size; i++)
      updateDictionary(classes[i]);
  }

  {
    std::vector<DataPropertyBranch_t*> data_properties = ontology_->data_property_graph_.getSafe();
    std::lock_guard<std::shared_timed_mutex> lock(ontology_->data_property_graph_.mutex_);
    graph_size = data_properties.size();
    for(size_t i = 0; i < graph_size; i++)
      updateDictionary(data_properties[i]);
  }

  {
    std::vector<ObjectPropertyBranch_t*> object_properties = ontology_->object_property_graph_.getSafe();
    std::lock_guard<std::shared_timed_mutex> lock(ontology_->object_property_graph_.mutex_);
    graph_size = object_properties.size();
    for(size_t i = 0; i < graph_size; i++)
      updateDictionary(object_properties[i]);
  }
}

void ReasonerDictionary::updateDictionary(ValuedNode* node)
{
  if (node->flags_.find("dico") == node->flags_.end())
  {
    split(node);
    createLowerCase(node);
    replaceQuote(node);
    node->flags_["dico"].push_back("true");
  }
}

void ReasonerDictionary::split(ValuedNode* node)
{
  size_t i, dic_size = 0;
  for(auto& it : node->dictionary_.spoken_)
  {
    std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];

    dic_size = it.second.size();
    for(i = 0; i < dic_size; i++)
    {
      std::string tmp = it.second[i];
      std::replace( tmp.begin(), tmp.end(), '_', ' ');
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
          it.second.push_back(tmp);
      std::replace( tmp.begin(), tmp.end(), '-', ' ');
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }

    for(i = 0; i < dic_size; i++)
    {
      std::string tmp = "";
      tmp = tmp + it.second[i][0];
      for(size_t char_i = 1; char_i < it.second[i].size(); char_i++)
      {
        if((it.second[i][char_i] >= 'A') && (it.second[i][char_i] <= 'Z'))
          if(it.second[i][char_i - 1] != ' ')
            tmp = tmp + ' ';
        tmp = tmp + it.second[i][char_i];
      }
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }
  }
}

void ReasonerDictionary::createLowerCase(ValuedNode* node)
{
  size_t i, dic_size = 0;
  for(auto& it : node->dictionary_.muted_)
  {
    dic_size = it.second.size();
    for(i = 0; i < dic_size; i++)
    {
      std::string tmp = "";
      tmp.resize(it.second[i].size());
      std::transform(it.second[i].begin(), it.second[i].end(), tmp.begin(), ::tolower);
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          it.second.push_back(tmp);
    }
  }

  for(auto& it : node->dictionary_.spoken_)
  {
    std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
    dic_size = it.second.size();
    for(i = 0; i < dic_size; i++)
    {
      std::string tmp = "";
      tmp.resize(it.second[i].size());
      std::transform(it.second[i].begin(), it.second[i].end(), tmp.begin(), ::tolower);
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }
  }
}

void ReasonerDictionary::replaceQuote(ValuedNode* node)
{
  size_t i, dic_size = 0;
  for (auto& it : node->dictionary_.spoken_)
  {
    std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
    dic_size = it.second.size();
    for(i = 0; i < dic_size; i++)
    {
      std::string tmp = it.second[i];
      tmp.erase(std::remove(tmp.begin(), tmp.end(), '\''), tmp.end());
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);

      tmp = it.second[i];
      std::replace( tmp.begin(), tmp.end(), '\'', ' ');
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }
  }

  for (auto& it : node->dictionary_.muted_)
  {
    std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
    dic_size = it.second.size();
    for(i = 0; i < dic_size; i++)
    {
      std::string tmp = it.second[i];
      tmp.erase(std::remove(tmp.begin(), tmp.end(), '\''), tmp.end());
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);

      tmp = it.second[i];
      std::replace( tmp.begin(), tmp.end(), '\'', ' ');
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }
  }
}

std::string ReasonerDictionary::getName()
{
  return "reasoner dictionary";
}

std::string ReasonerDictionary::getDesciption()
{
  return "This reasoner creates several alternative dictionaries to avoid too many restrictive labels.";
}

PLUGINLIB_EXPORT_CLASS(ReasonerDictionary, ReasonerInterface)

} // namespace ontologenius
