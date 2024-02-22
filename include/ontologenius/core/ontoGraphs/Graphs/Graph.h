#ifndef ONTOLOGENIUS_GRAPH_H
#define ONTOLOGENIUS_GRAPH_H

#include <exception>
#include <string>
#include <vector>
#include <map>
#include <mutex>  // For std::unique_lock
#include <shared_mutex>
#include <random>
#include <regex>

#include "ontologenius/core/ontoGraphs/BranchContainer/BranchContainerMap.h"
#include "ontologenius/core/ontoGraphs/BranchContainer/BranchContainerDyn.h"
#include "ontologenius/core/ontoGraphs/BranchContainer/BranchContainerSet.h"

#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h"

#include "ontologenius/core/Algorithms/LevenshteinDistance.h"

namespace ontologenius {

struct GraphException : public std::exception {
  std::string msg_;
  explicit GraphException(const std::string& msg) : msg_(msg) {}
  const char * what () const throw () {
    return msg_.c_str();
  }
};

template <typename B>
class Graph
{
  static_assert(std::is_base_of<ValuedNode,B>::value, "B must be derived from ValuedNode");
public:
  Graph() : language_("en") {}
  ~Graph()
  {
    for(auto& branch : all_branchs_)
      delete branch;
    all_branchs_.clear();
  }

  void setLanguage(const std::string& language) {language_ = language; }
  std::string getLanguage() {return language_; }

  const std::vector<B*>& get() { return this->all_branchs_; }
  const std::vector<B*>& getSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return this->all_branchs_;
  }

  virtual B* findBranchSafe(const std::string& name);
  virtual B* findBranch(const std::string& name);
  virtual B* findBranchSafe(index_t index);
  virtual B* findBranch(index_t index);
  B* findOrCreateBranch(const std::string& name);
  B* newDefaultBranch(const std::string& name);

  std::vector<std::string> getAll();
  std::vector<index_t> getAllIndex();

  index_t getIndex(const std::string& name);
  std::vector<index_t> getIndexes(const std::vector<std::string>& names);
  std::string getIdentifier(index_t index);
  std::vector<std::string> getIdentifiers(const std::vector<index_t>& indexes);

  template <typename T> bool touch(const T& value);

  bool addLang(const std::string& branch_str, const std::string& lang, const std::string& name);
  bool addLang(B* branch, const std::string& lang, const std::string& name);
  bool removeLang(const std::string& branch_str, const std::string& lang, const std::string& name);
  bool removeLang(B* branch, const std::string& lang, const std::string& name);

  template <typename T> std::string getName(const T& value, bool use_default = true);
  template <typename T> std::vector<std::string> getNames(const T& value, bool use_default = true);
  template <typename T> std::vector<std::string> getEveryNames(const T& value, bool use_default = true);

  template <typename T> std::unordered_set<T> find(const std::string& value, bool use_default = true);
  template <typename T> std::unordered_set<T> findSub(const std::string& value, bool use_default = true);
  template <typename T> std::unordered_set<T> findRegex(const std::string& regex, bool use_default = true);
  std::unordered_set<std::string> findFuzzy(const std::string& value, bool use_default = true, double threshold = 0.5);

  BranchContainerSet<B> container_;
  std::vector<B*> all_branchs_;

  std::string language_;

  mutable std::shared_timed_mutex mutex_;
  //use std::lock_guard<std::shared_timed_mutex> lock(mutex_); to WRITE A DATA
  //use std::shared_lock<std::shared_timed_mutex> lock(mutex_); to READ A DATA

public:

  inline void removeFromDictionary(std::map<std::string, std::vector<std::string>>& dictionary, const std::string& lang, const std::string& word)
  {
    if(dictionary.find(lang) != dictionary.end())
    {
      for(size_t i = 0; i < dictionary[lang].size();)
        if(dictionary[lang][i] == word)
          dictionary[lang].erase(dictionary[lang].begin() + i);
        else
          i++;
    }
  }

  template <class T>
  inline void removeFromVect(std::vector<T>& vect, const T& value)
  {
    for(size_t i = 0; i < vect.size();)
      if(vect[i] == value)
        vect.erase(vect.begin() + i);
      else
        i++;
  }
  
  template <class T>
  inline void removeFromElemVect(std::vector<Single_t<T>>& vect, const T& value)
  {
    for(size_t i = 0; i < vect.size();)
      if(vect[i].elem == value)
        vect.erase(vect.begin() + i);
      else
        i++;
  }

  template <class T>
  inline void removeFromElemVect(RelationsWithInductions<Single_t<T>>& vect, const T& value)
  {
    for(size_t i = 0; i < vect.size();)
      if(vect[i].elem == value)
        vect.erase(i);
      else
        i++;
  }

  template<typename T>
  inline void getInMap(T** ptr, const std::string& name, std::map<std::string, T*>& map)
  {
    if(*ptr != nullptr)
      return;

    auto it = map.find(name);
    if(it != map.end())
      *ptr = it->second;
  }

  template<typename C>
  inline bool conditionalPushBack(std::vector<C>& vect, const C& data)
  {
    if(std::find(vect.begin(), vect.end(), data) == vect.end())
    {
      vect.emplace_back(data);
      return true;
    }
    else
      return false;
  }

  template<typename C>
  inline bool conditionalPushBack(RelationsWithInductions<C>& vect, const C& data)
  {
    if(std::find(vect.begin(), vect.end(), data) == vect.end())
    {
      vect.emplace_back(data);
      return true;
    }
    else
      return false;
  }

  template<typename C>
  inline bool conditionalPushBack(std::vector<Single_t<C>>& vect, const Single_t<C>& data)
  {
    auto it = std::find(vect.begin(), vect.end(), data);
    if(it == vect.end())
    {
      vect.emplace_back(data);
      return true;
    }
    else if(it->infered && (data.infered == false))
      it->infered = false;
      
    return false;
  }

  template<typename C>
  inline bool conditionalPushBack(RelationsWithInductions<Single_t<C>>& vect, const Single_t<C>& data)
  {
    auto it = std::find(vect.begin(), vect.end(), data);
    if(it == vect.end())
    {
      vect.emplace_back(data);
      return true;
    }
    else if(it->infered && (data.infered == false))
      it->infered = false;
      
    return false;
  }

  bool insert(std::unordered_set<std::string>& set, ValuedNode* node) { return set.insert(node->value()).second; }
  bool insert(std::unordered_set<index_t>& set, ValuedNode* node) { return set.insert(node->get()).second; }
  bool insert(std::unordered_set<std::string>& set, LiteralNode* node) { return set.insert(node->value()).second; }
  bool insert(std::unordered_set<index_t>& set, LiteralNode* node) { return set.insert(node->get()).second; }

  bool compare(ValuedNode* node, const std::string& name) { return node->value() == name; }
  bool compare(ValuedNode* node, index_t index) { return node->get() == index; }
  bool compare(LiteralNode* node, const std::string& name) { return node->value() == name; }
  bool compare(LiteralNode* node, index_t index) { return node->get() == index; }

  template<template<typename> class C>
  C<B*> intersection(const std::unordered_set<B*>& set, const C<B*>& c)
  {
    C<B*> res;
    if(set.empty())
      return res;
      
    for(B* v : c)
    {
      if(set.find(v) != set.end())
        std::inserter(res, res.end()) = v;
    }
    return res;
  }

  template<template<typename> class C>
  B* firstIntersection(const std::unordered_set<B*>& set, const C<B*>& c)
  {
    if(set.empty())
      return nullptr;
      
    for(B* v : c)
    {
      if(set.find(v) != set.end())
        return v;
    }
    return nullptr;
  }

  template<template<typename> class C>
  C<B*> intersection(const std::unordered_set<B*>& set, const C<Single_t<B*>>& c)
  {
    C<B*> res;
    if(set.empty())
      return res;
      
    for(auto& v : c)
    {
      if(set.find(v.elem) != set.end())
        std::inserter(res, res.end()) = v.elem;
    }
    return res;
  }

  template<template<typename> class C>
  B* firstIntersection(const std::unordered_set<B*>& set, const C<Single_t<B*>>& c)
  {
    if(set.empty())
      return nullptr;

    for(auto& v : c)
    {
      if(set.find(v.elem) != set.end())
        return v.elem;
    }
    return nullptr;
  }
};

template <typename B>
B* Graph<B>::findBranchSafe(const std::string& name)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return container_.find(name);
}

template <typename B>
B* Graph<B>::findBranch(const std::string& name)
{
  return container_.find(name);
}

template <typename B>
B* Graph<B>::findBranchSafe(index_t index)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return container_.find(ValuedNode::table_.get(index));
}

template <typename B>
B* Graph<B>::findBranch(index_t index)
{
  return container_.find(ValuedNode::table_.get(index));
}

template <typename B>
B* Graph<B>::findOrCreateBranch(const std::string& name)
{
  B* branch = container_.find(name);
  if(branch == nullptr)
    branch = newDefaultBranch(name);
  return branch;
}

template <typename B>
B* Graph<B>::newDefaultBranch(const std::string& name)
{
  auto branch = new B(name);
  all_branchs_.push_back(branch);
  container_.insert(branch);
  return branch;
}

template <typename B>
std::vector<std::string> Graph<B>::getAll()
{
  std::vector<std::string> res;
  std::transform(all_branchs_.cbegin(), all_branchs_.cend(), std::back_inserter(res), [](auto branch){ return branch->value(); });
  return res;
}

template <typename B>
std::vector<index_t> Graph<B>::getAllIndex()
{
  std::vector<index_t> res;
  std::transform(all_branchs_.cbegin(), all_branchs_.cend(), std::back_inserter(res), [](auto branch){ return branch->get(); });
  return res;
}

template <typename B>
index_t Graph<B>::getIndex(const std::string& name)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  auto branch = container_.find(name);
  if(branch != nullptr)
    return branch->get();
  else
    return 0;
}

template <typename B>
std::vector<index_t> Graph<B>::getIndexes(const std::vector<std::string>& names)
{
  std::vector<index_t> res;
  std::transform(names.cbegin(), names.cend(), std::back_inserter(res), [this](const auto& name){ return getIndex(name); });
  return res; 
}

template <typename B>
std::string Graph<B>::getIdentifier(index_t index)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  if((index > 0) && (index < (index_t)ValuedNode::table_.size()))
    return ValuedNode::table_[index];
  else
    return "";
}

template <typename B>
std::vector<std::string> Graph<B>::getIdentifiers(const std::vector<index_t>& indexes)
{
  std::vector<std::string> res;
  std::transform(indexes.cbegin(), indexes.cend(), std::back_inserter(res), [this](const auto& index){ return getIdentifier(index); });
  return res; 
}

template <typename B>
template <typename T>
bool Graph<B>::touch(const T& value)
{
  return (findBranchSafe(value) != nullptr);
}

template <typename B>
bool Graph<B>::addLang(const std::string& branch_str, const std::string& lang, const std::string& name)
{
  B* branch = this->findBranchSafe(branch_str);
  return addLang(branch, lang, name);
}

template <typename B>
bool Graph<B>::addLang(B* branch, const std::string& lang, const std::string& name)
{
  if(branch != nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(this->mutex_);
    branch->setSteadyDictionary(lang.substr(1), name);
    branch->updated_ = true;
    return true;
  }
  else
    return false;
}

template <typename B>
bool Graph<B>::removeLang(const std::string& indiv, const std::string& lang, const std::string& name)
{
  B* branch = findBranchSafe(indiv);
  return removeLang(branch, lang, name);
}

template <typename B>
bool Graph<B>::removeLang(B* branch, const std::string& lang, const std::string& name)
{
  if(branch != nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);

    auto lang_id = lang.substr(1);
    removeFromDictionary(branch->dictionary_.spoken_, lang_id, name);
    removeFromDictionary(branch->dictionary_.muted_, lang_id, name);
    removeFromDictionary(branch->steady_dictionary_.spoken_, lang_id, name);
    removeFromDictionary(branch->steady_dictionary_.muted_, lang_id, name);

    return true;
  }
  else
    return false;
}

template <typename B>
template <typename T>
std::string Graph<B>::getName(const T& value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  B* branch = findBranch(value);

  if(branch != nullptr)
  {
    if(branch->dictionary_.spoken_.find(language_) != branch->dictionary_.spoken_.end())
    {
      if(branch->dictionary_.spoken_[language_].size())
      {
        std::unordered_set<size_t> tested;
        std::random_device rd;
        std::mt19937 gen(rd());

        size_t dic_size = branch->dictionary_.spoken_[language_].size();
        std::uniform_int_distribution<> dis(0, dic_size - 1);

        while(tested.size() < dic_size)
        {
          size_t myIndex = dis(gen);
          std::string word = branch->dictionary_.spoken_[language_][myIndex];
          if(word.find("_") == std::string::npos)
            return word;
          tested.insert(myIndex);
        }
        return branch->dictionary_.spoken_[language_][0];
      }
      else if(use_default)
        return branch->value();
    }
    else if(use_default)
      return branch->value();
  }

  return "";
}

template <typename B>
template <typename T>
std::vector<std::string> Graph<B>::getNames(const T& value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  B* branch = findBranch(value);
  
  std::vector<std::string> res;
  if(branch != nullptr)
  {
    if(branch->dictionary_.spoken_.find(language_) != branch->dictionary_.spoken_.end())
      res = branch->dictionary_.spoken_[language_];
    else if(use_default)
      res.push_back(branch->value());
  }

  return res;
}

template <typename B>
template <typename T>
std::vector<std::string> Graph<B>::getEveryNames(const T& value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  B* branch = findBranch(value);
  
  std::vector<std::string> res;
  if(branch != nullptr)
  {
    if(branch->dictionary_.spoken_.find(language_) != branch->dictionary_.spoken_.end())
      res = branch->dictionary_.spoken_[language_];
    else if(use_default)
      res.push_back(branch->value());

    if(branch->dictionary_.muted_.find(language_) != branch->dictionary_.muted_.end())
    {
      std::vector<std::string> muted = branch->dictionary_.muted_[language_];
      res.insert(res.end(), muted.begin(), muted.end());
    }
  }

  return res;
}

template <typename D>
bool fullComparator(D* branch, const std::string& value, const std::string& lang, bool use_default)
{
  if(use_default)
    if(branch->value() == value)
      return true;

  if(branch->dictionary_.spoken_.find(lang) != branch->dictionary_.spoken_.end())
    if(std::any_of(branch->dictionary_.spoken_[lang].begin(), branch->dictionary_.spoken_[lang].end(), [value](auto& word){ return word == value; }))
      return true;

  if(branch->dictionary_.muted_.find(lang) != branch->dictionary_.muted_.end())
    if(std::any_of(branch->dictionary_.muted_[lang].begin(), branch->dictionary_.muted_[lang].end(), [value](auto& word){ return word == value; }))
      return true;

  return false;
}

template <typename D>
bool comparator(D* branch, const std::string& value, const std::string& lang, bool use_default)
{
  std::smatch match;

  if(use_default)
  {
    std::regex regex("\\b(" + branch->value() + ")([^ ]*)");
    if(std::regex_search(value, match, regex))
      return true;
  }

  if(branch->dictionary_.spoken_.find(lang) != branch->dictionary_.spoken_.end())
    for(auto& word : branch->dictionary_.spoken_[lang])
    {
      std::regex regex("\\b(" + word + ")([^ ]*)");
      if(std::regex_search(value, match, regex))
        return true;
    }

  if(branch->dictionary_.muted_.find(lang) != branch->dictionary_.muted_.end())
    for(auto& word : branch->dictionary_.muted_[lang])
    {
      std::regex regex("\\b(" + word + ")([^ ]*)");
      if(std::regex_search(value, match, regex))
        return true;
    }
  return false;
}

template <typename D>
bool comparatorRegex(D* branch, const std::string& regex, const std::string& lang, bool use_default)
{
  std::regex base_regex(regex);
  std::smatch match;

  if(use_default)
  {
    std::string tmp = branch->value();
    if(std::regex_match(tmp, match, base_regex))
      return true;
  }

  if(branch->dictionary_.spoken_.find(lang) != branch->dictionary_.spoken_.end())
    for(auto& word : branch->dictionary_.spoken_[lang])
      if(std::regex_match(word, match, base_regex))
        return true;

  if(branch->dictionary_.muted_.find(lang) != branch->dictionary_.muted_.end())
    for(auto& word : branch->dictionary_.muted_[lang])
      if(std::regex_match(word, match, base_regex))
        return true;
  return false;
}

template <typename B>
template <typename T>
std::unordered_set<T> Graph<B>::find(const std::string& value, bool use_default)
{
  std::unordered_set<T> res;
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  std::vector<B*> branchs = this->container_.find(&fullComparator<B>, value, language_, use_default);
  for(auto& branch : branchs)
    insert(res, branch);

  return res;
}

template <typename B>
template <typename T>
std::unordered_set<T> Graph<B>::findSub(const std::string& value, bool use_default)
{
  std::unordered_set<T> res;
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  std::vector<B*> branchs = this->container_.find(&comparator<B>, value, language_, use_default);
  for(auto& branch : branchs)
    insert(res, branch);

  return res;
}

template <typename B>
template <typename T>
std::unordered_set<T> Graph<B>::findRegex(const std::string& regex, bool use_default)
{
  std::unordered_set<T> res;
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  std::vector<B*> branchs = this->container_.find(&comparatorRegex<B>, regex, language_, use_default);
  for(auto& branch : branchs)
    insert(res, branch);

  return res;
}

template <typename B>
std::unordered_set<std::string> Graph<B>::findFuzzy(const std::string& value, bool use_default, double threshold)
{
  double lower_cost = 100000;
  double tmp_cost;
  std::unordered_set<std::string> res;

  LevenshteinDistance dist;

  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  for(auto branch : this->all_branchs_)
  {
    if(use_default)
      if((tmp_cost = dist.get(branch-> value(), value)) <= lower_cost)
      {
        if(tmp_cost != lower_cost)
        {
          lower_cost = tmp_cost;
          res.clear();
        }
        res.insert(branch->value());
      }

    if(branch->dictionary_.spoken_.find(language_) != branch->dictionary_.spoken_.end())
      for(auto& word : branch->dictionary_.spoken_[language_])
        if((tmp_cost = dist.get(word, value)) <= lower_cost)
        {
          if(tmp_cost != lower_cost)
          {
            lower_cost = tmp_cost;
            res.clear();
          }
          res.insert(word);
        }

    if(branch->dictionary_.muted_.find(language_) != branch->dictionary_.muted_.end())
      for(auto& word : branch->dictionary_.muted_[language_])
        if((tmp_cost = dist.get(word, value)) <= lower_cost)
        {
          if(tmp_cost != lower_cost)
          {
            lower_cost = tmp_cost;
            res.clear();
          }
          res.insert(word);
        }
  }

  if(lower_cost > threshold)
    res.clear();

  return res;
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_GRAPH_H
