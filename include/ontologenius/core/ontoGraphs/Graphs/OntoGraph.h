#ifndef ONTOLOGENIUS_ONTOGRAPH_H
#define ONTOLOGENIUS_ONTOGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>
#include <random>
#include <regex>

#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"

#include "ontologenius/core/Algorithms/LevenshteinDistance.h"

/*
This file use CRTP (curiously recurring template pattern)
be really carreful of how you use it
*/

namespace ontologenius {

template <typename B>
class OntoGraph : public Graph<B>
{
  static_assert(std::is_base_of<Branch_t<B>,B>::value, "B must be derived from Branch_t<B>");
public:
  OntoGraph() {}
  ~OntoGraph();

  virtual void close() override;

  std::unordered_set<std::string> getDown(const std::string& value, int depth = -1);
  std::unordered_set<index_t> getDown(index_t value, int depth = -1) { return getDownId(value, depth); }
  std::unordered_set<std::string> getUp(const std::string& value, int depth = -1);
  std::unordered_set<index_t> getUp(index_t value, int depth = -1) { return getUpId(value, depth); }
  std::unordered_set<index_t> getDownId(const std::string& value, int depth = -1);
  std::unordered_set<index_t> getDownId(index_t value, int depth = -1);
  std::unordered_set<index_t> getUpId(const std::string& value, int depth = -1);
  std::unordered_set<index_t> getUpId(index_t value, int depth = -1);
  std::string getName(const std::string& value, bool use_default = true);
  std::string getName(index_t value, bool use_default = true);
  std::vector<std::string> getNames(const std::string& value, bool use_default = true);
  std::vector<std::string> getNames(index_t value, bool use_default = true);
  std::vector<std::string> getEveryNames(const std::string& value, bool use_default = true);
  std::vector<std::string> getEveryNames(index_t value, bool use_default = true);
  template <typename T> std::unordered_set<T> find(const std::string& value, bool use_default = true);
  template <typename T> std::unordered_set<T> findSub(const std::string& value, bool use_default = true);
  template <typename T> std::unordered_set<T> findRegex(const std::string& regex, bool use_default = true);
  std::unordered_set<std::string> findFuzzy(const std::string& value, bool use_default = true, double threshold = 0.5);
  bool touch(const std::string& value);
  bool touch(index_t value);

  void getDown(B* branch, std::unordered_set<std::string>& res, int depth = -1, unsigned int current_depth = 0);
  void getUp(B* branch, std::unordered_set<std::string>& res, int depth = -1, unsigned int current_depth = 0);
  void getDown(B* branch, std::unordered_set<index_t>& res, int depth = -1, unsigned int current_depth = 0);
  void getUp(B* branch, std::unordered_set<index_t>& res, int depth = -1, unsigned int current_depth = 0);

  std::unordered_set<B*> getDownPtrSafe(B* branch, int depth = -1);
  void getDownPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth = 0);
  inline void getDownPtr(B* branch, std::unordered_set<B*>& res);
  std::unordered_set<B*> getUpPtrSafe(B* branch, int depth = -1);
  void getUpPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth = 0);
  inline void getUpPtr(B* branch, std::unordered_set<B*>& res);

  template<typename T> std::unordered_set<T> select(const std::unordered_set<T>& on, const T& selector)
  {
    std::unordered_set<T> res;
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);

    for(auto& it : on)
    {
      std::unordered_set<T> tmp = getUp(it);
      if(tmp.find(selector) != tmp.end())
        res.insert(it);
    }
    return res;
  }

  std::vector<B*> get() override
  {
    return all_branchs_;
  }

  std::vector<B*> getSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);

    return all_branchs_;
  }

  std::vector<std::string> getAll()
  {
    std::vector<std::string> res;
    std::transform(all_branchs_.cbegin(), all_branchs_.cend(), std::back_inserter(res), [](auto branch){ return branch->value(); });
    return res;
  }

protected:
  std::map<std::string, B*> branchs_;
  std::map<std::string, B*> roots_;
  std::vector<B*> all_branchs_;

  std::map<std::string, B*> tmp_mothers_;
  
  void amIA(B** me, std::map<std::string, B*>& vect, const std::string& value, bool erase = true);

  void mitigate(B* branch);
  std::vector<B*> intersection(const std::unordered_set<B*>& set, const std::vector<B*>& vect);
  std::vector<B*> intersection(const std::unordered_set<B*>& set, const std::vector<Single_t<B*>>& vect);
  void eraseFromVector(std::vector<B*>& vect, B* branch);
  void insert(std::unordered_set<std::string>& set, ValuedNode* node) { set.insert(node->value()); }
  void insert(std::unordered_set<index_t>& set, ValuedNode* node) { set.insert(node->get()); }
  void insert(std::unordered_set<std::string>& set, LiteralNode* node) { set.insert(node->value()); }
  void insert(std::unordered_set<index_t>& set, LiteralNode* node) { set.insert(node->get()); }

private:
  std::string getName(B* branch, bool use_default);
  std::vector<std::string> getNames(B* branch, bool use_default);
  std::vector<std::string> getEveryNames(B* branch, bool use_default = true);
};

template <typename B>
OntoGraph<B>::~OntoGraph()
{
  for(auto& branch : all_branchs_)
    delete branch;

  branchs_.clear();
  roots_.clear();
  all_branchs_.clear();
}

template <typename B>
void OntoGraph<B>::close()
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<B>::mutex_);

  roots_.insert(tmp_mothers_.begin(), tmp_mothers_.end());

  tmp_mothers_.clear();

  //link();

  std::transform(roots_.cbegin(), roots_.cend(), std::back_inserter(all_branchs_), [](auto map_it){ return map_it.second; });
  std::transform(branchs_.cbegin(), branchs_.cend(), std::back_inserter(all_branchs_), [](auto map_it){ return map_it.second; });

  this->container_.load(all_branchs_);
}

template <typename B>
std::unordered_set<std::string> OntoGraph<B>::getDown(const std::string& value, int depth)
{
  std::unordered_set<std::string> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);

  if(branch != nullptr)
    getDown(branch, res, depth);

  return res;
}

template <typename B>
std::unordered_set<std::string> OntoGraph<B>::getUp(const std::string& value, int depth)
{
  std::unordered_set<std::string> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);

  if(branch != nullptr)
    getUp(branch, res, depth);

  return res;
}

template <typename B>
std::unordered_set<index_t> OntoGraph<B>::getDownId(const std::string& value, int depth)
{
  std::unordered_set<index_t> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);

  if(branch != nullptr)
    getDown(branch, res, depth);

  return res;
}

template <typename B>
std::unordered_set<index_t> OntoGraph<B>::getDownId(index_t value, int depth)
{
  std::unordered_set<index_t> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(ValuedNode::table_.get(value));

  if(branch != nullptr)
    getDown(branch, res, depth);

  return res;
}

template <typename B>
std::unordered_set<index_t> OntoGraph<B>::getUpId(const std::string& value, int depth)
{
  std::unordered_set<index_t> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);

  if(branch != nullptr)
    getUp(branch, res, depth);

  return res;
}

template <typename B>
std::unordered_set<index_t> OntoGraph<B>::getUpId(index_t value, int depth)
{
  std::unordered_set<index_t> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(ValuedNode::table_.get(value));

  if(branch != nullptr)
    getUp(branch, res, depth);

  return res;
}

template <typename B>
std::string OntoGraph<B>::getName(const std::string& value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);

  return getName(branch, use_default);
}

template <typename B>
std::string OntoGraph<B>::getName(index_t value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(ValuedNode::table_.get(value));

  return getName(branch, use_default);
}

template <typename B>
std::string OntoGraph<B>::getName(B* branch, bool use_default)
{
  std::string res = "";

  if(branch != nullptr)
  {
    if(branch->dictionary_.spoken_.find(this->language_) != branch->dictionary_.spoken_.end())
    {
      if(branch->dictionary_.spoken_[this->language_].size())
      {
        std::unordered_set<size_t> tested;
        std::random_device rd;
        std::mt19937 gen(rd());

        size_t dic_size = branch->dictionary_.spoken_[this->language_].size();
        std::uniform_int_distribution<> dis(0, dic_size - 1);

        while(tested.size() < dic_size)
        {
          size_t myIndex = dis(gen);
          std::string word = branch->dictionary_.spoken_[this->language_][myIndex];
          if(word.find("_") == std::string::npos)
          {
            res = word;
            break;
          }
          tested.insert(myIndex);
        }
        if(res == "")
          res = branch->dictionary_.spoken_[this->language_][0];
      }
      else if(use_default)
        res = branch->value();
    }
    else if(use_default)
      res = branch->value();
  }

  return res;
}

template <typename B>
std::vector<std::string> OntoGraph<B>::getNames(const std::string& value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);
  return getNames(branch, use_default);
}

template <typename B>
std::vector<std::string> OntoGraph<B>::getNames(index_t value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(ValuedNode::table_.get(value));
  return getNames(branch, use_default);
}

template <typename B>
std::vector<std::string> OntoGraph<B>::getNames(B* branch, bool use_default)
{
  std::vector<std::string> res;

  if(branch != nullptr)
  {
    if(branch->dictionary_.spoken_.find(this->language_) != branch->dictionary_.spoken_.end())
      res = branch->dictionary_.spoken_[this->language_];
    else if(use_default)
      res.push_back(branch->value());
  }

  return res;
}

template <typename B>
std::vector<std::string> OntoGraph<B>::getEveryNames(const std::string& value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);
  return getEveryNames(branch, use_default);
}

template <typename B>
std::vector<std::string> OntoGraph<B>::getEveryNames(index_t value, bool use_default)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(ValuedNode::table_.get(value));
  return getEveryNames(branch, use_default);
}

template <typename B>
std::vector<std::string> OntoGraph<B>::getEveryNames(B* branch, bool use_default)
{
  std::vector<std::string> res;
  if(branch != nullptr)
  {
    if(branch->dictionary_.spoken_.find(this->language_) != branch->dictionary_.spoken_.end())
      res = branch->dictionary_.spoken_[this->language_];
    else if(use_default)
      res.push_back(branch->value());

    if(branch->dictionary_.muted_.find(this->language_) != branch->dictionary_.muted_.end())
    {
      std::vector<std::string> muted = branch->dictionary_.muted_[this->language_];
      res.insert(res.end(), muted.begin(), muted.end());
    }
  }

  return res;
}

template <typename B>
bool OntoGraph<B>::touch(const std::string& value)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  return (this->container_.find(value) != nullptr);
}

template <typename B>
bool OntoGraph<B>::touch(index_t value)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  return (this->container_.find(ValuedNode::table_.get(value)) != nullptr);
}

template <typename B>
void OntoGraph<B>::amIA(B** me, std::map<std::string, B*>& vect, const std::string& value, bool erase)
{
  if(*me == nullptr)
  {
    auto it = vect.find(value);
    if(it != vect.end())
    {
      *me = it->second;
      if(erase)
        vect.erase(it);
    }
  }
}

template <typename B>
void OntoGraph<B>::getDown(B* branch, std::unordered_set<std::string>& res, int depth, unsigned int current_depth)
{
  if(current_depth <= (unsigned int)depth)
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
    current_depth++;
    if(res.insert(branch->value()).second)
      for(auto& child : branch->childs_)
        getDown(child.elem, res, depth, current_depth);
  }
}

template <typename B>
void OntoGraph<B>::getUp(B* branch, std::unordered_set<std::string>& res, int depth, unsigned int current_depth)
{
  if(current_depth <= (unsigned int)depth)
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
    current_depth++;

    if(res.insert(branch->value()).second)
      for(auto& mother : branch->mothers_)
        getUp(mother.elem, res, depth, current_depth);
  }
}

template <typename B>
void OntoGraph<B>::getDown(B* branch, std::unordered_set<index_t>& res, int depth, unsigned int current_depth)
{
  if(current_depth <= (unsigned int)depth)
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
    current_depth++;
    if(res.insert(branch->get()).second)
      for(auto& child : branch->childs_)
        getDown(child.elem, res, depth, current_depth);
  }
}

template <typename B>
void OntoGraph<B>::getUp(B* branch, std::unordered_set<index_t>& res, int depth, unsigned int current_depth)
{
  if(current_depth <= (unsigned int)depth)
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
    current_depth++;
    if(res.insert(branch->get()).second)
      for(auto& mother : branch->mothers_)
        getUp(mother.elem, res, depth, current_depth);
  }
}

template <typename B>
std::unordered_set<B*> OntoGraph<B>::getDownPtrSafe(B* branch, int depth)
{
  std::unordered_set<B*> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  getDownPtr(branch, res, depth);
  return res;
}

template <typename B>
void OntoGraph<B>::getDownPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth)
{
  if(current_depth <= (unsigned int)depth)
  {
    current_depth++;
    if(res.insert(branch).second)
      for(auto& it : branch->childs_)
        if(res.find(it.elem) == res.end())
          getDownPtr(it.elem, res, depth, current_depth);
  }
}

template <typename B>
void OntoGraph<B>::getDownPtr(B* branch, std::unordered_set<B*>& res)
{
  if(res.insert(branch).second)
    for(auto& it : branch->childs_)
      if(res.find(it.elem) == res.end())
        getDownPtr(it.elem, res);
}

template <typename B>
std::unordered_set<B*> OntoGraph<B>::getUpPtrSafe(B* branch, int depth)
{
  std::unordered_set<B*> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  getUpPtr(branch, res, depth);
  return res;
}

template <typename B>
void OntoGraph<B>::getUpPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth)
{
  if(current_depth <= (unsigned int)depth)
  {
    current_depth++;
    if(res.insert(branch).second)
    {
      for(auto& mother : branch->mothers_)
        if(res.find(mother.elem) == res.end())
          getUpPtr(mother.elem, res, depth, current_depth);
    }
  }
}

template <typename B>
void OntoGraph<B>::getUpPtr(B* branch, std::unordered_set<B*>& res)
{
  if(res.insert(branch).second)
    for(auto& it : branch->mothers_)
      if(res.find(it.elem) == res.end())
        getUpPtr(it.elem, res);
}

template <typename D>
bool fullComparator(D* branch, const std::string& value, const std::string& lang, bool use_default)
{
  if(use_default)
    if(branch->value() == value)
      return true;

  if(branch->dictionary_.spoken_.find(lang) != branch->dictionary_.spoken_.end())
    if(std::any_of(branch->dictionary_.spoken_[lang].begin(), branch->dictionary_.spoken_[lang].end(), [value](auto word){ return word == value; }))
      return true;

  if(branch->dictionary_.muted_.find(lang) != branch->dictionary_.muted_.end())
    if(std::any_of(branch->dictionary_.muted_[lang].begin(), branch->dictionary_.muted_[lang].end(), [value](auto word){ return word == value; }))
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
std::unordered_set<T> OntoGraph<B>::find(const std::string& value, bool use_default)
{
  std::unordered_set<T> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  std::vector<B*> branchs = this->container_.find(&fullComparator<B>, value, this->language_, use_default);
  for(auto& branch : branchs)
    insert(res, branch);

  return res;
}

template <typename B>
template <typename T>
std::unordered_set<T> OntoGraph<B>::findSub(const std::string& value, bool use_default)
{
  std::unordered_set<T> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  std::vector<B*> branchs = this->container_.find(&comparator<B>, value, this->language_, use_default);
  for(auto& branch : branchs)
    insert(res, branch);

  return res;
}

template <typename B>
template <typename T>
std::unordered_set<T> OntoGraph<B>::findRegex(const std::string& regex, bool use_default)
{
  std::unordered_set<T> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  std::vector<B*> branchs = this->container_.find(&comparatorRegex<B>, regex, this->language_, use_default);
  for(auto& branch : branchs)
    insert(res, branch);

  return res;
}

template <typename B>
std::unordered_set<std::string> OntoGraph<B>::findFuzzy(const std::string& value, bool use_default, double threshold)
{
  double lower_cost = 100000;
  double tmp_cost;
  std::unordered_set<std::string> res;

  LevenshteinDistance dist;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  for(auto branch : all_branchs_)
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

    if(branch->dictionary_.spoken_.find(this->language_) != branch->dictionary_.spoken_.end())
      for(auto& word : branch->dictionary_.spoken_[this->language_])
        if((tmp_cost = dist.get(word, value)) <= lower_cost)
        {
          if(tmp_cost != lower_cost)
          {
            lower_cost = tmp_cost;
            res.clear();
          }
          res.insert(word);
        }

    if(branch->dictionary_.muted_.find(this->language_) != branch->dictionary_.muted_.end())
      for(auto& word : branch->dictionary_.muted_[this->language_])
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

template <typename B>
void OntoGraph<B>::mitigate(B* branch)
{
  std::vector<Single_t<B*>> childs = branch->childs_;
  for(Single_t<B*>& child : childs)
  {
    std::unordered_set<B*> up;
    getUpPtr(child.elem, up);
    std::vector<B*> inter = intersection(up, childs);
    if(inter.size() > 1)
    {
      this->removeFromElemVect(child.elem->mothers_, branch);
      this->removeFromElemVect(branch->childs_, child.elem);
    }
  }

  std::vector<Single_t<B*>> mothers = branch->mothers_;
  for(Single_t<B*>& mother : mothers)
  {
    std::unordered_set<B*> down;
    getDownPtr(mother.elem, down);
    std::vector<B*> inter = intersection(down, mothers);
    if(inter.size() > 1)
    {
      this->removeFromElemVect(branch->mothers_, mother.elem);
      this->removeFromElemVect(mother.elem->childs_, branch);
    }
  }
}

template <typename B>
std::vector<B*> OntoGraph<B>::intersection(const std::unordered_set<B*>& set, const std::vector<B*>& vect)
{
  std::vector<B*> res;
  for(B* v : vect)
  {
    if(set.find(v) != set.end())
      res.push_back(v);
  }
  return res;
}

template <typename B>
std::vector<B*> OntoGraph<B>::intersection(const std::unordered_set<B*>& set, const std::vector<Single_t<B*>>& vect)
{
  std::vector<B*> res;
  for(const Single_t<B*>& v : vect)
  {
    if(set.find(v.elem) != set.end())
      res.push_back(v.elem);
  }
  return res;
}

template <typename B>
void OntoGraph<B>::eraseFromVector(std::vector<B*>& vect, B* branch)
{
  for(size_t i = 0; i < vect.size(); i++)
  {
    if(vect[i] == branch)
    {
      vect.erase(vect.begin() + i);
      break;
    }
  }
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOGRAPH_H
