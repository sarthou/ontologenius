#include "ontologenius/core/reasoner/plugins/ReasonerDictionary.h"

#include <pluginlib/class_list_macros.hpp>

namespace ontologenius {

void ReasonerDictionary::setParameter(const std::string& name, const std::string& value)
{
  if(name == "use_id")
    use_id_ = (value == "true");
}

void ReasonerDictionary::postReason()
{
  {
    std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
    std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.get();
    for(auto elem : indivs)
      updateDictionary(elem);
  }

  {
    std::vector<ClassBranch_t*> classes = ontology_->class_graph_.getSafe();
    std::lock_guard<std::shared_timed_mutex> lock(ontology_->class_graph_.mutex_);
    for(auto elem : classes)
      updateDictionary(elem);
  }

  {
    std::vector<DataPropertyBranch_t*> data_properties = ontology_->data_property_graph_.getSafe();
    std::lock_guard<std::shared_timed_mutex> lock(ontology_->data_property_graph_.mutex_);
    for(auto elem : data_properties)
      updateDictionary(elem);
  }

  {
    std::vector<ObjectPropertyBranch_t*> object_properties = ontology_->object_property_graph_.getSafe();
    std::lock_guard<std::shared_timed_mutex> lock(ontology_->object_property_graph_.mutex_);
    for(auto elem : object_properties)
      updateDictionary(elem);
  }
}

void ReasonerDictionary::updateDictionary(ValuedNode* node)
{
  if (node->flags_.find("dico") == node->flags_.end())
  {
    if(use_id_)
      setId(node);
    split(node);
    createLowerCase(node);
    replaceQuote(node);
    node->flags_["dico"].push_back("true");
  }
}

void ReasonerDictionary::setId(ValuedNode* node)
{
  if(node->dictionary_.spoken_["en"].size() == 0)
    node->dictionary_.spoken_["en"] = std::vector<std::string>(1, node->value());
}

void ReasonerDictionary::split(ValuedNode* node)
{
  for(auto& it : node->dictionary_.spoken_)
  {
    std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];

    for(auto& word : it.second)
    {
      std::string tmp = word;
      std::replace( tmp.begin(), tmp.end(), '_', ' ');
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
          it.second.push_back(tmp);
      
      std::replace( tmp.begin(), tmp.end(), '-', ' ');
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }

    for(auto& word : it.second)
    {
      std::string tmp = "";
      tmp += word[0];
      for(size_t char_i = 1; char_i < word.size(); char_i++)
      {
        if((word[char_i] >= 'A') && (word[char_i] <= 'Z'))
          if(word[char_i - 1] != ' ')
            tmp += ' ';
        tmp += word[char_i];
      }
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }
  }
}

void ReasonerDictionary::createLowerCase(ValuedNode* node)
{
  for(auto& it : node->dictionary_.muted_)
  {
    for(auto& word : it.second)
    {
      if(word.size())
      {
        std::string tmp = "";
        tmp.resize(word.size());
        std::transform(word.begin(), word.end(), tmp.begin(), ::tolower);
        if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          it.second.push_back(tmp);
      }
    }
  }

  for(auto& it : node->dictionary_.spoken_)
  {
    std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
    for(auto& word : it.second)
    {
      std::string tmp = "";
      tmp.resize(word.size());
      std::transform(word.begin(), word.end(), tmp.begin(), ::tolower);
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }
  }
}

void ReasonerDictionary::replaceQuote(ValuedNode* node)
{
  for (auto& it : node->dictionary_.spoken_)
  {
    std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
    for(auto& word : it.second)
    {
      std::string tmp = word;
      tmp.erase(std::remove(tmp.begin(), tmp.end(), '\''), tmp.end());
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);

      tmp = word;
      std::replace( tmp.begin(), tmp.end(), '\'', ' ');
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);
    }
  }

  for (auto& it : node->dictionary_.muted_)
  {
    std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
    for(auto& word : it.second)
    {
      std::string tmp = word;
      tmp.erase(std::remove(tmp.begin(), tmp.end(), '\''), tmp.end());
      if (std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
        if (std::find(muted->begin(), muted->end(), tmp) == muted->end())
          muted->push_back(tmp);

      tmp = word;
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

std::string ReasonerDictionary::getDescription()
{
  return "This reasoner creates several alternative dictionaries to avoid too many restrictive labels.";
}

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerDictionary, ontologenius::ReasonerInterface)
