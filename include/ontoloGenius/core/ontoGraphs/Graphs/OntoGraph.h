#ifndef ONTOLOGENIUS_ONTOGRAPH_H
#define ONTOLOGENIUS_ONTOGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>
#include <random>
#include <regex>

#include "ontoloGenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/Branch.h"

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

  void close();

  std::unordered_set<std::string> getDown(const std::string& value, int depth = -1);
  std::unordered_set<std::string> getUp(const std::string& value, int depth = -1);
  std::unordered_set<uint32_t> getDownIdSafe(const std::string& value, int depth = -1);
  std::unordered_set<uint32_t> getUpIdSafe(const std::string& value, int depth = -1);
  std::string getName(const std::string& value);
  std::vector<std::string> getNames(const std::string& value);
  std::unordered_set<std::string> find(const std::string& value);
  std::unordered_set<std::string> findSub(const std::string& value);
  std::unordered_set<std::string> findRegex(const std::string& regex);
  bool touch(const std::string& value);

  void getDown(B* branch, std::unordered_set<std::string>& res, int depth = -1, unsigned int current_depth = 0);
  void getUp(B* branch, std::unordered_set<std::string>& res, int depth = -1, unsigned int current_depth = 0);
  void getDownIdSafe(B* branch, std::unordered_set<uint32_t>& res, int depth = -1, unsigned int current_depth = 0);
  void getUpIdSafe(B* branch, std::unordered_set<uint32_t>& res, int depth = -1, unsigned int current_depth = 0);

  std::unordered_set<B*> getDownPtrSafe(B* branch, int depth = -1);
  void getDownPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth = 0);
  inline void getDownPtr(B* branch, std::unordered_set<B*>& res);
  std::unordered_set<B*> getUpPtrSafe(B* branch, int depth = -1);
  void getUpPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth = 0);
  inline void getUpPtr(B* branch, std::unordered_set<B*>& res);

  std::vector<B*> get()
  {
    return all_branchs_;
  }

  std::vector<B*> getSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);

    return all_branchs_;
  }

protected:
  std::map<std::string, B*> branchs_;
  std::map<std::string, B*> roots_;
  std::vector<B*> all_branchs_;

  std::map<std::string, B*> tmp_mothers_;

  int depth_;

  void link();
  void add_family(B* branch, uint8_t family);
  void amIA(B** me, std::map<std::string, B*>& vect, const std::string& value, bool erase = true);
  void isMyMother(B* me, const std::string& mother, std::map<std::string, B*>& vect, bool& find);
};

template <typename B>
OntoGraph<B>::~OntoGraph()
{
  for(size_t i = 0; i < all_branchs_.size(); i++)
    delete all_branchs_[i];

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

  for(auto& it : roots_)
    all_branchs_.push_back(it.second);
  for(auto& it : branchs_)
    all_branchs_.push_back(it.second);

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
std::unordered_set<uint32_t> OntoGraph<B>::getDownIdSafe(const std::string& value, int depth)
{
  std::unordered_set<uint32_t> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);

  if(branch != nullptr)
    getDownIdSafe(branch, res, depth);

  return res;
}

template <typename B>
std::unordered_set<uint32_t> OntoGraph<B>::getUpIdSafe(const std::string& value, int depth)
{
  std::unordered_set<uint32_t> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);

  if(branch != nullptr)
    getUpIdSafe(branch, res, depth);

  return res;
}


template <typename B>
std::string OntoGraph<B>::getName(const std::string& value)
{
  std::string res = "";

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);
  if(branch != nullptr)
  {
    if(branch->dictionary_.find(this->language_) != branch->dictionary_.end())
      if(branch->dictionary_[this->language_].size())
      {
        std::unordered_set<size_t> tested;
        std::random_device rd;
        std::mt19937 gen(rd());

        size_t dic_size = branch->dictionary_[this->language_].size();
        std::uniform_int_distribution<> dis(0, dic_size - 1);

        while(tested.size() < dic_size)
        {
          size_t myIndex = dis(gen);
          std::string word = branch->dictionary_[this->language_][myIndex];
          if(word.find("_") == std::string::npos)
          {
            res = word;
            break;
          }
          tested.insert(myIndex);
        }
        if(res == "")
          res = branch->dictionary_[this->language_][0];
      }
      else
        res = value;
    else
      res = value;
  }

  return res;
}

template <typename B>
std::vector<std::string> OntoGraph<B>::getNames(const std::string& value)
{
  std::vector<std::string> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);
  if(branch != nullptr)
  {
    if(branch->dictionary_.find(this->language_) != branch->dictionary_.end())
      res = branch->dictionary_[this->language_];
    else
      res.push_back(value);
  }

  return res;
}

template <typename B>
bool OntoGraph<B>::touch(const std::string& value)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->container_.find(value);
  if(branch != nullptr)
    return true;
  else
    return false;
}

template <typename B>
void OntoGraph<B>::link()
{
  depth_ = 0;

  uint8_t nb_root_family = roots_.size();
  size_t root_i = 0;
  for(auto& it : roots_)
  {
    it.second->family = 256/(nb_root_family+1) * root_i;
    for(size_t i = 0; i < it.second->childs_.size(); i++)
      add_family(it.second->childs_[i], it.second->family);
    root_i++;
  }
}

template <typename B>
void OntoGraph<B>::add_family(B* branch, uint8_t family)
{
  branch->family += family/branch->nb_mothers_;
  for(size_t i = 0; i < branch->childs_.size(); i++)
  {
    depth_++;
    if(depth_ < 20)
      add_family(branch->childs_[i], family/branch->nb_mothers_);
    depth_--;
  }
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
void OntoGraph<B>::isMyMother(B* me, const std::string& mother, std::map<std::string, B*>& vect, bool& find)
{
  if(find)
    return;

  auto it = vect.find(mother);
  if(it != vect.end())
  {
    bool loop = false;
    for(B* mothers : it->second->mothers_)
      if(mothers == me)
        loop = true;

    if(loop == false)
    {
      it->second->childs_.push_back(me);
      me->setSteady_mother(it->second);
    }

    find = true;
  }
}

template <typename B>
void OntoGraph<B>::getDown(B* branch, std::unordered_set<std::string>& res, int depth, unsigned int current_depth)
{
  if(current_depth < (unsigned int)depth)
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
    size_t size = branch->childs_.size();
    current_depth++;
    for(size_t i = 0; i < size; i++)
      if(res.find(branch->childs_[i]->value()) == res.end())
        getDown(branch->childs_[i], res, depth, current_depth);
  }

  res.insert(branch->value());
}

template <typename B>
void OntoGraph<B>::getUp(B* branch, std::unordered_set<std::string>& res, int depth, unsigned int current_depth)
{
  if(current_depth < (unsigned int)depth)
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
    size_t size = branch->mothers_.size();
    current_depth++;
    for(size_t i = 0; i < size; i++)
      if(res.find(branch->mothers_[i]->value()) == res.end())
        getUp(branch->mothers_[i], res, depth, current_depth);
  }

  res.insert(branch->value());
}

template <typename B>
void OntoGraph<B>::getDownIdSafe(B* branch, std::unordered_set<uint32_t>& res, int depth, unsigned int current_depth)
{
  if(current_depth < (unsigned int)depth)
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
    size_t size = branch->childs_.size();
    current_depth++;
    for(size_t i = 0; i < size; i++)
      getDownIdSafe(branch->childs_[i], res, depth, current_depth);
  }

  res.insert(branch->get());
}

template <typename B>
void OntoGraph<B>::getUpIdSafe(B* branch, std::unordered_set<uint32_t>& res, int depth, unsigned int current_depth)
{
  if(current_depth < (unsigned int)depth)
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
    size_t size = branch->mothers_.size();
    current_depth++;
    for(size_t i = 0; i < size; i++)
      getUpIdSafe(branch->mothers_[i], res, depth, current_depth);
  }

  res.insert(branch->get());
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
    res.insert(branch);

    for(B* it : branch->childs_)
      if(res.find(it) == res.end())
        getDownPtr(it, res, depth, current_depth);
  }
}

template <typename B>
void OntoGraph<B>::getDownPtr(B* branch, std::unordered_set<B*>& res)
{
  res.insert(branch);

  for(B* it : branch->childs_)
    if(res.find(it) == res.end())
      getDownPtr(it, res);
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
    res.insert(branch);

    size_t size = branch->mothers_.size();
    for(size_t i = 0; i < size; i++)
      if(res.find(branch->mothers_[i]) == res.end())
        getUpPtr(branch->mothers_[i], res, depth, current_depth);
  }

}

template <typename B>
void OntoGraph<B>::getUpPtr(B* branch, std::unordered_set<B*>& res)
{
  res.insert(branch);

  for(B* it : branch->mothers_)
    if(res.find(it) == res.end())
      getUpPtr(it, res);
}

template <typename D>
bool fullComparator(D* branch, const std::string& value, const std::string& lang)
{
  if(branch->dictionary_.find(lang) != branch->dictionary_.end())
    for(size_t i = 0; i < branch->dictionary_[lang].size(); i++)
      if(branch->dictionary_[lang][i] == value)
        return true;
  return false;
}

template <typename D>
bool comparator(D* branch, const std::string& value, const std::string& lang)
{
  std::smatch match;

  if(branch->dictionary_.find(lang) != branch->dictionary_.end())
    for(size_t i = 0; i < branch->dictionary_[lang].size(); i++)
    {
      std::regex regex(".*" + branch->dictionary_[lang][i] + ".*");
      if(std::regex_match(value, match, regex))
        return true;
    }
  return false;
}

template <typename D>
bool comparatorRegex(D* branch, const std::string& regex, const std::string& lang)
{
  std::regex base_regex(regex);
  std::smatch match;

  if(branch->dictionary_.find(lang) != branch->dictionary_.end())
    for(size_t i = 0; i < branch->dictionary_[lang].size(); i++)
      if(std::regex_match(branch->dictionary_[lang][i], match, base_regex))
        return true;
  return false;
}

template <typename B>
std::unordered_set<std::string> OntoGraph<B>::find(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  std::vector<B*> branch = this->container_.find(&fullComparator<B>, value, this->language_);
  for(size_t i = 0; i < branch.size(); i++)
    res.insert(branch[i]->value());

  return res;
}

template <typename B>
std::unordered_set<std::string> OntoGraph<B>::findSub(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  std::vector<B*> branch = this->container_.find(&comparator<B>, value, this->language_);
  for(size_t i = 0; i < branch.size(); i++)
    res.insert(branch[i]->value());

  return res;
}

template <typename B>
std::unordered_set<std::string> OntoGraph<B>::findRegex(const std::string& regex)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  std::vector<B*> branch = this->container_.find(&comparatorRegex<B>, regex, this->language_);
  for(size_t i = 0; i < branch.size(); i++)
    res.insert(branch[i]->value());

  return res;
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOGRAPH_H
