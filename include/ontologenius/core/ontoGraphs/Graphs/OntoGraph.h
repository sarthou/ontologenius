#ifndef ONTOLOGENIUS_ONTOGRAPH_H
#define ONTOLOGENIUS_ONTOGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>

#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"

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
  explicit OntoGraph(IndividualGraph* individual_graph) : individual_graph_(individual_graph) {}
  ~OntoGraph() {}

  template <typename T> std::unordered_set<T> getDown(const T& value, int depth = -1);
  std::unordered_set<index_t> getDown(index_t value, int depth = -1) { return getDownId(value, depth); }
  std::unordered_set<std::string> getUp(const std::string& value, int depth = -1);
  std::unordered_set<index_t> getUp(index_t value, int depth = -1) { return getUpId(value, depth); }
  template <typename T> std::unordered_set<index_t> getDownId(const T& value, int depth = -1);
  template <typename T> std::unordered_set<index_t> getUpId(const T& value, int depth = -1);

  template <typename T> bool existInInheritance(B* branch, const T& selector);

  template <typename T> void getDownSafe(B* branch, std::unordered_set<T>& res, int depth = -1, unsigned int current_depth = 0);
  template <typename T> void getUpSafe(B* branch, std::unordered_set<T>& res, int depth = -1, unsigned int current_depth = 0);
  template <typename T> void getDown(B* branch, std::unordered_set<T>& res, int depth = -1, unsigned int current_depth = 0);
  template <typename T> void getUp(B* branch, std::unordered_set<T>& res, int depth = -1, unsigned int current_depth = 0);

  std::unordered_set<B*> getDownPtrSafe(B* branch, int depth = -1);
  void getDownPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth = 0);
  inline void getDownPtr(B* branch, std::unordered_set<B*>& res);
  std::unordered_set<B*> getUpPtrSafe(B* branch, int depth = -1);
  void getUpPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth = 0);
  inline void getUpPtr(B* branch, std::unordered_set<B*>& res);

  template <typename T> std::unordered_set<T> getDisjoint(const T& value);
  void getDisjoint(B* branch, std::unordered_set<B*>& res);
  void getLocalDisjoint(B* branch, std::unordered_set<B*>& res);
  B* isDisjoint(const std::unordered_set<B*>& set_base, B* branch);
  B* isDisjoint(const std::unordered_set<B*>& set_base, const std::unordered_set<B*>& ups);

  bool addInheritage(const std::string& branch_base, const std::string& branch_inherited);
  bool addInheritage(B* branch, B* inherited);
  std::vector<std::pair<std::string, std::string>> removeInheritage(const std::string& branch_base, const std::string& branch_inherited);
  std::vector<std::pair<std::string, std::string>> removeInheritage(B* branch, B* inherited);
  bool removeInheritage(IndividualBranch_t* indiv, ClassBranch_t* class_branch, std::vector<std::pair<std::string, std::string>>& explanations);

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

  template<typename T>
  std::vector<std::pair<std::string, std::string>> removeInductions(B* indiv_from, RelationsWithInductions<Single_t<T>>& relations, size_t relation_index, const std::string& property)
  {
    std::vector<std::pair<std::string, std::string>> explanations;

    auto relation = relations[relation_index];
    auto indiv_on = relation.elem;

    for(size_t i = 0; i < relations.has_induced_object_relations[relation_index]->triplets.size(); )
    {
      auto& triplet = relations.has_induced_object_relations[relation_index]->triplets[i];               
      auto tmp = individual_graph_->removeRelation(triplet.subject,
                                                   triplet.predicate,
                                                   triplet.object,
                                                   true);
      if(tmp.second)
      {
        explanations.emplace_back("[DEL]" + triplet.subject->value() + "|" +
                                          triplet.predicate->value() + "|" +
                                          triplet.object->value(),
                                  "[DEL]" + indiv_from->value() + "|" + property + "|" + indiv_on->value());
        explanations.insert(explanations.end(), tmp.first.begin(), tmp.first.end());
      }
      else
        i++; // we enter in this case if the induced relation has later been stated and can thus not be removed automatically
    }

    for(size_t i = 0; i < relations.has_induced_inheritance_relations[relation_index]->triplets.size(); )
    {
      auto& triplet = relations.has_induced_inheritance_relations[relation_index]->triplets[i];

      // DO it on object relations

      std::vector<std::pair<std::string, std::string>> tmp;
      std::lock_guard<std::shared_timed_mutex> lock(individual_graph_->mutex_);
      if(individual_graph_->removeInheritage(triplet.subject, triplet.object, tmp, true))
      {
        explanations.emplace_back("[DEL]" + triplet.subject->value() + "|isA|" +
                                          triplet.object->value(),
                                  "[DEL]" + indiv_from->value() + "|" + property + "|" + indiv_on->value());
        explanations.insert(explanations.end(), tmp.begin(), tmp.end());
      }
      else
        i++; // we enter in this case if the induced relation has later been stated and can thus not be removed automatically
    }

    return explanations;
  }

protected:
  IndividualGraph* individual_graph_;
  
  void amIA(B** me, std::map<std::string, B*>& vect, const std::string& value, bool erase = true);

  void mitigate(B* branch);
};

template <typename B>
template <typename T>
std::unordered_set<T> OntoGraph<B>::getDown(const T& value, int depth)
{
  std::unordered_set<T> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->findBranch(value);

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
template <typename T>
std::unordered_set<index_t> OntoGraph<B>::getDownId(const T& value, int depth)
{
  std::unordered_set<index_t> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->findBranch(value);

  if(branch != nullptr)
    getDown(branch, res, depth);

  return res;
}

template <typename B>
template <typename T>
std::unordered_set<index_t> OntoGraph<B>::getUpId(const T& value, int depth)
{
  std::unordered_set<index_t> res;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->findBranch(value);

  if(branch != nullptr)
    getUp(branch, res, depth);

  return res;
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
template <typename T>
bool OntoGraph<B>::existInInheritance(B* branch, const T& selector)
{
  if(this->compare(branch, selector))
    return true;
  else
    return std::any_of(branch->mothers_.begin(), branch->mothers_.end(), [selector, this](const auto& mother){ return existInInheritance(mother.elem, selector); } );
}

template <typename B>
template <typename T>
void OntoGraph<B>::getDownSafe(B* branch, std::unordered_set<T>& res, int depth, unsigned int current_depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  getDown(branch, res, depth, current_depth);
}

template <typename B>
template <typename T>
void OntoGraph<B>::getDown(B* branch, std::unordered_set<T>& res, int depth, unsigned int current_depth)
{
  if(current_depth <= (unsigned int)depth)
  {
    current_depth++;
    if(this->insert(res, branch))
      for(auto& child : branch->childs_)
        getDown(child.elem, res, depth, current_depth);
  }
}

template <typename B>
template <typename T>
void OntoGraph<B>::getUpSafe(B* branch, std::unordered_set<T>& res, int depth, unsigned int current_depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  getUp(branch, res, depth, current_depth);
}

template <typename B>
template <typename T>
void OntoGraph<B>::getUp(B* branch, std::unordered_set<T>& res, int depth, unsigned int current_depth)
{
  if(current_depth <= (unsigned int)depth)
  {
    current_depth++;

    if(this->insert(res, branch))
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
        getDownPtr(it.elem, res, depth, current_depth);
  }
}

template <typename B>
void OntoGraph<B>::getDownPtr(B* branch, std::unordered_set<B*>& res)
{
  if(res.insert(branch).second)
    for(auto& it : branch->childs_)
      getDownPtr(it.elem, res);
}

template <typename B>
std::unordered_set<B*> OntoGraph<B>::getUpPtrSafe(B* branch, int depth)
{
  std::unordered_set<B*> res;
  res.reserve(branch->mothers_.size() + 1);
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  getUpPtr(branch, res, depth);
  return res;
}

template <typename B>
void OntoGraph<B>::getUpPtr(B* branch, std::unordered_set<B*>& res, int depth, unsigned int current_depth)
{
  if(current_depth < (unsigned int)depth)
  {
    current_depth++;
    if(res.insert(branch).second)
      for(auto& mother : branch->mothers_)
        getUpPtr(mother.elem, res, depth, current_depth);
  }
  else if(current_depth == (unsigned int)depth)
    res.insert(branch);
}

template <typename B>
void OntoGraph<B>::getUpPtr(B* branch, std::unordered_set<B*>& res)
{
  if(res.insert(branch).second)
    for(auto& it : branch->mothers_)
      getUpPtr(it.elem, res);
}

template <typename B>
template <typename T>
std::unordered_set<T> OntoGraph<B>::getDisjoint(const T& value)
{
  std::unordered_set<T> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);

  B* branch = this->findBranch(value);
  if(branch != nullptr)
  {
    std::unordered_set<B*> ups;
    getUpPtr(branch, ups);
    for(auto up : ups)
      for(auto& disjoint : up->disjoints_)
        getDown(disjoint.elem, res);
  } 

  return res;
}

template <typename B>
void OntoGraph<B>::getDisjoint(B* branch, std::unordered_set<B*>& res)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);

  if(branch != nullptr)
  {
    std::unordered_set<B*> ups;
    getUpPtr(branch, ups);
    for(auto up : ups)
      for(auto& disjoint : up->disjoints_)
        getDownPtr(disjoint.elem, res);
  }
}

template <typename B>
void OntoGraph<B>::getLocalDisjoint(B* branch, std::unordered_set<B*>& res)
{
  if(branch != nullptr)
  {
    std::unordered_set<B*> ups;
    getUpPtr(branch, ups);
    for(auto up : ups)
      for(auto& disjoint : up->disjoints_)
        res.insert(disjoint.elem);
  }
}

template <typename B>
B* OntoGraph<B>::isDisjoint(const std::unordered_set<B*>& set_base, B* branch)
{
  std::unordered_set<B*> ups;
  getUpPtr(branch, ups);
  return isDisjoint(set_base, ups);
}

template <typename B>
B* OntoGraph<B>::isDisjoint(const std::unordered_set<B*>& set_base, const std::unordered_set<B*>& ups)
{
  std::unordered_set<B*> disjoints;
  for(auto elem : set_base)
    getLocalDisjoint(elem, disjoints);
  return this->firstIntersection(ups, disjoints);
}

// both branches can be created automatically
template <typename B>
bool OntoGraph<B>::addInheritage(const std::string& branch_base, const std::string& branch_inherited)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<B>::mutex_);
  B* branch = this->findOrCreateBranch(branch_base);
  if(branch != nullptr)
  {
    B* inherited = this->findOrCreateBranch(branch_inherited);
    if(inherited != nullptr)
      return addInheritage(branch, inherited);
  }

  return false;
}

template <typename B>
bool OntoGraph<B>::addInheritage(B* branch, B* inherited)
{
  if((branch != nullptr) && (inherited != nullptr))
  {
    this->conditionalPushBack(branch->mothers_, Single_t<B*>(inherited));
    this->conditionalPushBack(inherited->childs_, Single_t<B*>(branch));
    branch->updated_ = true;
    inherited->updated_ = true;
    mitigate(branch);

    std::unordered_set<B*> downs;
    getDownPtr(branch, downs);
    for(auto down : downs)
      down->updated_ = true; // propagate update

    return true; // TODO verify that multi inheritances are compatible
  }
  else
    return false;
}

template <typename B>
std::vector<std::pair<std::string, std::string>> OntoGraph<B>::removeInheritage(const std::string& branch_base, const std::string& branch_inherited)
{
  B* branch_base_ptr = this->findBranchSafe(branch_base);
  B* branch_inherited_ptr = this->findBranchSafe(branch_inherited);

  if(branch_base_ptr == nullptr)
  {
    throw GraphException("The concept " + branch_base + " does not exist");
    return std::vector<std::pair<std::string, std::string>>();
  }
  if(branch_inherited_ptr == nullptr)
  {
    throw GraphException("The concept " + branch_inherited + " does not exist");
    return std::vector<std::pair<std::string, std::string>>();
  }

  std::lock_guard<std::shared_timed_mutex> lock(this->mutex_);
  return removeInheritage(branch_base_ptr, branch_inherited_ptr);
}

template <typename B>
std::vector<std::pair<std::string, std::string>> OntoGraph<B>::removeInheritage(B* branch, B* inherited)
{
  for(size_t i = 0; i < branch->mothers_.size(); i++)
  {
    if(branch->mothers_[i].elem == inherited)
    {
      auto explanations = removeInductions(branch, branch->mothers_, i, "isA");

      branch->mothers_.erase(i);

      this->removeFromElemVect(inherited->childs_, branch);
      branch->updated_ = true;
      inherited->updated_ = true;
      return explanations;
    }
  }

  return {};
}

template <typename B>
void OntoGraph<B>::mitigate(B* branch)
{
  std::vector<Single_t<B*>> childs = branch->childs_;
  for(Single_t<B*>& child : childs)
  {
    std::unordered_set<B*> up;
    getUpPtr(child.elem, up);
    std::vector<B*> inter = this->intersection(up, childs);
    if(inter.size() > 1)
    {
      this->removeFromElemVect(child.elem->mothers_, branch);
      this->removeFromElemVect(branch->childs_, child.elem);
    }
  }

  RelationsWithInductions<Single_t<B*>>& mothers = branch->mothers_;
  for(Single_t<B*>& mother : mothers)
  {
    std::unordered_set<B*> down;
    getDownPtr(mother.elem, down);
    std::vector<B*> inter = this->intersection(down, mothers.relations);
    if(inter.size() > 1)
    {
      this->removeFromElemVect(branch->mothers_, mother.elem);
      this->removeFromElemVect(mother.elem->childs_, branch);
    }
  }
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOGRAPH_H
