#include "ontologenius/core/reasoner/plugins/ReasonerDictionary.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  void ReasonerDictionary::setParameter(const std::string& name, const std::string& value)
  {
    if(name == "use_id")
      use_id_ = (value == "true");
  }

  void ReasonerDictionary::postReason()
  {
    {
      const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
      const std::vector<IndividualBranch*> indivs = ontology_->individual_graph_.get();
      for(auto* elem : indivs)
        updateDictionary(elem);
    }

    {
      const std::vector<ClassBranch*> classes = ontology_->class_graph_.getSafe();
      const std::lock_guard<std::shared_timed_mutex> lock(ontology_->class_graph_.mutex_);
      for(auto* elem : classes)
        updateDictionary(elem);
    }

    {
      const std::vector<DataPropertyBranch*> data_properties = ontology_->data_property_graph_.getSafe();
      const std::lock_guard<std::shared_timed_mutex> lock(ontology_->data_property_graph_.mutex_);
      for(auto* elem : data_properties)
        updateDictionary(elem);
    }

    {
      const std::vector<ObjectPropertyBranch*> object_properties = ontology_->object_property_graph_.getSafe();
      const std::lock_guard<std::shared_timed_mutex> lock(ontology_->object_property_graph_.mutex_);
      for(auto* elem : object_properties)
        updateDictionary(elem);
    }
  }

  void ReasonerDictionary::updateDictionary(ValuedNode* node)
  {
    if(node->flags_.find("dico") == node->flags_.end())
    {
      if(use_id_)
        setId(node);
      split(node);
      createLowerCase(node);
      replaceQuote(node);
      node->flags_["dico"].emplace_back("true");
    }
  }

  void ReasonerDictionary::setId(ValuedNode* node)
  {
    if(node->dictionary_.spoken_["en"].empty())
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
        std::replace(tmp.begin(), tmp.end(), '_', ' ');
        if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
            it.second.push_back(tmp);

        std::replace(tmp.begin(), tmp.end(), '-', ' ');
        if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
            muted->push_back(tmp);
      }

      for(auto& word : it.second)
      {
        std::string tmp;
        tmp += word[0];
        for(size_t char_i = 1; char_i < word.size(); char_i++)
        {
          if((word[char_i] >= 'A') && (word[char_i] <= 'Z'))
            if(word[char_i - 1] != ' ')
              tmp += ' ';
          tmp += word[char_i];
        }
        if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
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
        if(word.empty() == false)
        {
          std::string tmp;
          tmp.resize(word.size());
          std::transform(word.begin(), word.end(), tmp.begin(), ::tolower);
          if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
            it.second.push_back(tmp);
        }
      }
    }

    for(auto& it : node->dictionary_.spoken_)
    {
      std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
      for(auto& word : it.second)
      {
        std::string tmp;
        tmp.resize(word.size());
        std::transform(word.begin(), word.end(), tmp.begin(), ::tolower);
        if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
            muted->push_back(tmp);
      }
    }
  }

  void ReasonerDictionary::replaceQuote(ValuedNode* node)
  {
    for(auto& it : node->dictionary_.spoken_)
    {
      std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
      for(auto& word : it.second)
      {
        std::string tmp = word;
        tmp.erase(std::remove(tmp.begin(), tmp.end(), '\''), tmp.end());
        if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
            muted->push_back(tmp);

        tmp = word;
        std::replace(tmp.begin(), tmp.end(), '\'', ' ');
        if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
            muted->push_back(tmp);
      }
    }

    for(auto& it : node->dictionary_.muted_)
    {
      std::vector<std::string>* muted = &node->dictionary_.muted_[it.first];
      for(auto& word : it.second)
      {
        std::string tmp = word;
        tmp.erase(std::remove(tmp.begin(), tmp.end(), '\''), tmp.end());
        if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
            muted->push_back(tmp);

        tmp = word;
        std::replace(tmp.begin(), tmp.end(), '\'', ' ');
        if(std::find(it.second.begin(), it.second.end(), tmp) == it.second.end())
          if(std::find(muted->begin(), muted->end(), tmp) == muted->end())
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
