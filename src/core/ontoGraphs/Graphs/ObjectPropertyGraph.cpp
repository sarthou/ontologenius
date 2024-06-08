#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"

namespace ontologenius {

  ObjectPropertyGraph::ObjectPropertyGraph(IndividualGraph* individual_graph,
                                           ClassGraph* class_graph) : OntoGraph(individual_graph),
                                                                      class_graph_(class_graph)
  {}

  ObjectPropertyGraph::ObjectPropertyGraph(const ObjectPropertyGraph& other,
                                           IndividualGraph* individual_graph,
                                           ClassGraph* class_graph) : OntoGraph(individual_graph),
                                                                      class_graph_(class_graph)
  {
    language_ = other.language_;

    for(const auto& branch : other.all_branchs_)
    {
      auto* prop_branch = new ObjectPropertyBranch(branch->value());
      all_branchs_.push_back(prop_branch);
    }

    this->container_.load(all_branchs_);
  }

  ObjectPropertyBranch* ObjectPropertyGraph::add(const std::string& value, ObjectPropertyVectors_t& property_vectors)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch>::mutex_);
    /**********************
    ** Mothers
    **********************/
    ObjectPropertyBranch* me = findOrCreateBranch(value);

    // for all my mothers
    for(auto& mother : property_vectors.mothers_)
    {
      ObjectPropertyBranch* mother_branch = findOrCreateBranch(mother.elem);

      conditionalPushBack(mother_branch->childs_, ObjectPropertyElement(me, mother.probability, true));
      conditionalPushBack(me->mothers_, ObjectPropertyElement(mother_branch, mother.probability));
    }

    /**********************
    ** Disjoints
    **********************/
    // for all my disjoints
    for(auto& disjoint : property_vectors.disjoints_)
    {
      ObjectPropertyBranch* disjoint_branch = findOrCreateBranch(disjoint.elem);

      conditionalPushBack(me->disjoints_, ObjectPropertyElement(disjoint_branch, disjoint.probability));
      conditionalPushBack(disjoint_branch->disjoints_, ObjectPropertyElement(me, disjoint.probability, true));
    }

    /**********************
    ** Inverses
    **********************/
    // for all my inverses
    for(auto& inverse : property_vectors.inverses_)
    {
      ObjectPropertyBranch* inverse_branch = findOrCreateBranch(inverse.elem);

      conditionalPushBack(me->inverses_, ObjectPropertyElement(inverse_branch, inverse.probability));
      conditionalPushBack(inverse_branch->inverses_, ObjectPropertyElement(me, inverse.probability, true));
    }

    /**********************
    ** Domains
    **********************/
    // for all my domains
    for(auto& domain : property_vectors.domains_)
    {
      ClassBranch* domain_branch = class_graph_->findOrCreateBranch(domain.elem);

      conditionalPushBack(me->domains_, ClassElement(domain_branch, domain.probability));
    }

    /**********************
    ** Ranges
    **********************/
    // for all my ranges
    for(auto& range : property_vectors.ranges_)
    {
      ClassBranch* range_branch = class_graph_->findOrCreateBranch(range.elem);

      conditionalPushBack(me->ranges_, ClassElement(range_branch, range.probability));
    }

    /**********************
    ** Language and properties
    **********************/
    me->properties_.apply(property_vectors.properties_);
    me->annotation_usage_ = me->annotation_usage_ || property_vectors.annotation_usage_;
    me->setSteadyDictionary(property_vectors.dictionary_);
    me->setSteadyMutedDictionary(property_vectors.muted_dictionary_);

    /**********************
    ** Chain axiom
    **********************/
    for(auto& chain_i : property_vectors.chains_)
    {
      if(chain_i.empty())
        continue;

      std::vector<ObjectPropertyBranch*> chain;
      ObjectPropertyBranch* first = nullptr;

      for(auto& link : chain_i)
      {
        ObjectPropertyBranch* next = findOrCreateBranch(link);

        if(first == nullptr)
          first = next;
        else
          chain.push_back(next);
      }

      chain.push_back(me);
      first->chains_.push_back(chain);
      me->str_chains_.push_back(chain_i);
    }

    mitigate(me);
    return me;
  }

  void ObjectPropertyGraph::add(std::vector<std::string>& disjoints)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch>::mutex_);

    for(size_t disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
    {
      // I need to find myself
      ObjectPropertyBranch* me = findOrCreateBranch(disjoints[disjoints_i]);

      // for all my disjoints ...
      for(size_t disjoints_j = 0; disjoints_j < disjoints.size(); disjoints_j++)
      {
        //... excepted me
        if(disjoints_i != disjoints_j)
        {
          ObjectPropertyBranch* disjoint_branch = findOrCreateBranch(disjoints[disjoints_j]);

          conditionalPushBack(me->disjoints_, ObjectPropertyElement(disjoint_branch));
          conditionalPushBack(disjoint_branch->disjoints_, ObjectPropertyElement(me, 1.0, true));
        }
      }
    }
  }

  std::unordered_set<std::string> ObjectPropertyGraph::getInverse(const std::string& value)
  {
    std::unordered_set<std::string> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch>::mutex_);

    ObjectPropertyBranch* branch = container_.find(value);
    if(branch != nullptr)
      for(auto& inverse : branch->inverses_)
        getDown(inverse.elem, res);

    return res;
  }

  std::unordered_set<index_t> ObjectPropertyGraph::getInverse(index_t value)
  {
    std::unordered_set<index_t> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch>::mutex_);

    ObjectPropertyBranch* branch = container_.find(ValuedNode::table.get(value));
    if(branch != nullptr)
      for(auto& inverse : branch->inverses_)
        getDown(inverse.elem, res);

    return res;
  }

  std::unordered_set<std::string> ObjectPropertyGraph::getDomain(const std::string& value, size_t depth)
  {
    std::unordered_set<std::string> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch>::mutex_);

    ObjectPropertyBranch* branch = container_.find(value);
    std::unordered_set<ObjectPropertyBranch*> up_trace;
    if(branch != nullptr)
      getDomain(branch, depth, res, up_trace);

    return res;
  }

  std::unordered_set<index_t> ObjectPropertyGraph::getDomain(index_t value, size_t depth)
  {
    std::unordered_set<index_t> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch>::mutex_);

    ObjectPropertyBranch* branch = container_.find(ValuedNode::table.get(value));
    std::unordered_set<ObjectPropertyBranch*> up_trace;
    if(branch != nullptr)
      getDomain(branch, depth, res, up_trace);

    return res;
  }

  void ObjectPropertyGraph::getDomainPtr(ObjectPropertyBranch* branch, std::unordered_set<ClassBranch*>& res, size_t depth)
  {
    std::unordered_set<ObjectPropertyBranch*> up_trace;
    if(branch != nullptr)
      getDomainPtr(branch, depth, res, up_trace);
  }

  template<typename T>
  void ObjectPropertyGraph::getDomain(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<T>& res, std::unordered_set<ObjectPropertyBranch*>& up_trace)
  {
    for(auto& domain : branch->domains_)
      class_graph_->getDown(domain.elem, res, depth);

    for(auto& mother : branch->mothers_)
      if(up_trace.insert(mother.elem).second)
        getDomain(mother.elem, depth, res, up_trace);
  }

  void ObjectPropertyGraph::getDomainPtr(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<ClassBranch*>& res, std::unordered_set<ObjectPropertyBranch*>& up_trace)
  {
    for(auto& domain : branch->domains_)
      class_graph_->getDownPtr(domain.elem, res, (int)depth);

    for(auto& mother : branch->mothers_)
      if(up_trace.insert(mother.elem).second)
        getDomainPtr(mother.elem, depth, res, up_trace);
  }

  std::unordered_set<std::string> ObjectPropertyGraph::getRange(const std::string& value, size_t depth)
  {
    std::unordered_set<std::string> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch>::mutex_);

    ObjectPropertyBranch* branch = container_.find(value);
    std::unordered_set<ObjectPropertyBranch*> up_trace;
    if(branch != nullptr)
      getRange(branch, depth, res, up_trace);

    return res;
  }

  std::unordered_set<index_t> ObjectPropertyGraph::getRange(index_t value, size_t depth)
  {
    std::unordered_set<index_t> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch>::mutex_);

    ObjectPropertyBranch* branch = container_.find(ValuedNode::table.get(value));
    std::unordered_set<ObjectPropertyBranch*> up_trace;
    if(branch != nullptr)
      getRange(branch, depth, res, up_trace);

    return res;
  }

  void ObjectPropertyGraph::getRangePtr(ObjectPropertyBranch* branch, std::unordered_set<ClassBranch*>& res, size_t depth)
  {
    std::unordered_set<ObjectPropertyBranch*> up_trace;
    if(branch != nullptr)
      getRangePtr(branch, depth, res, up_trace);
  }

  template<typename T>
  void ObjectPropertyGraph::getRange(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<T>& res, std::unordered_set<ObjectPropertyBranch*>& up_trace)
  {
    for(auto& range : branch->ranges_)
      class_graph_->getDown(range.elem, res, depth);

    for(auto& mother : branch->mothers_)
      if(up_trace.insert(mother.elem).second)
        getRange(mother.elem, depth, res, up_trace);
  }

  void ObjectPropertyGraph::getRangePtr(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<ClassBranch*>& res, std::unordered_set<ObjectPropertyBranch*>& up_trace)
  {
    for(auto& range : branch->ranges_)
      class_graph_->getDownPtr(range.elem, res, (int)depth);

    for(auto& mother : branch->mothers_)
      if(up_trace.insert(mother.elem).second)
        getRangePtr(mother.elem, depth, res, up_trace);
  }

  void ObjectPropertyGraph::getDomainAndRangePtr(ObjectPropertyBranch* branch, std::unordered_set<ClassBranch*>& domains, std::unordered_set<ClassBranch*>& ranges, size_t depth)
  {
    std::unordered_set<ObjectPropertyBranch*> up_trace;
    if(branch != nullptr)
      getDomainAndRangePtr(branch, depth, domains, ranges, up_trace);
  }

  void ObjectPropertyGraph::getDomainAndRangePtr(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<ClassBranch*>& domains, std::unordered_set<ClassBranch*>& ranges, std::unordered_set<ObjectPropertyBranch*>& up_trace)
  {
    for(auto& domain : branch->domains_)
      class_graph_->getDownPtr(domain.elem, domains, (int)depth);

    for(auto& range : branch->ranges_)
      class_graph_->getDownPtr(range.elem, ranges, (int)depth);

    for(auto& mother : branch->mothers_)
      if(up_trace.insert(mother.elem).second)
        getDomainAndRangePtr(mother.elem, depth, domains, ranges, up_trace);
  }

  bool ObjectPropertyGraph::addInverseOf(const std::string& from, const std::string& on)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    ObjectPropertyBranch* from_branch = container_.find(from);
    ObjectPropertyBranch* on_branch = container_.find(on);
    if((from_branch == nullptr) && (on_branch == nullptr))
      return false;
    else
    {
      if(from_branch == nullptr)
        from_branch = newDefaultBranch(from);
      else if(on_branch == nullptr)
        on_branch = newDefaultBranch(on);
    }

    conditionalPushBack(from_branch->inverses_, ObjectPropertyElement(on_branch, 1.0));
    conditionalPushBack(on_branch->inverses_, ObjectPropertyElement(from_branch, 1.0, true));
    return true;
  }

  bool ObjectPropertyGraph::removeInverseOf(const std::string& from, const std::string& on)
  {
    ObjectPropertyBranch* from_branch = container_.find(from);
    if(from_branch == nullptr)
      return false;

    ObjectPropertyBranch* on_branch = container_.find(on);
    if(on_branch == nullptr)
      return false;

    for(size_t i = 0; i < from_branch->inverses_.size(); i++)
    {
      if(from_branch->inverses_[i].elem == on_branch)
      {
        from_branch->inverses_.erase(from_branch->inverses_.begin() + (long)i);
        break;
      }
    }

    for(size_t i = 0; i < on_branch->inverses_.size(); i++)
    {
      if(on_branch->inverses_[i].elem == from_branch)
      {
        on_branch->inverses_.erase(on_branch->inverses_.begin() + (long)i);
        break;
      }
    }
    return true;
  }

  bool ObjectPropertyGraph::isIrreflexive(const std::string& prop)
  {
    ObjectPropertyBranch* branch = container_.find(prop);
    if(branch == nullptr)
      return false;
    else
      return isIrreflexive(branch);
  }

  bool ObjectPropertyGraph::isIrreflexive(ObjectPropertyBranch* prop)
  {
    if(prop->properties_.irreflexive_property_)
      return true;
    else
    {
      for(auto& mother : prop->mothers_)
      {
        if(isIrreflexive(mother.elem))
          return true;
      }
    }

    return false;
  }

  bool ObjectPropertyGraph::isAsymetric(const std::string& prop)
  {
    ObjectPropertyBranch* branch = container_.find(prop);
    if(branch == nullptr)
      return false;
    else
      return isAsymetric(branch);
  }

  bool ObjectPropertyGraph::isAsymetric(ObjectPropertyBranch* prop)
  {
    if(prop->properties_.antisymetric_property_)
      return true;
    else
    {
      for(auto& mother : prop->mothers_)
      {
        if(isIrreflexive(mother.elem))
          return true;
      }
    }

    return false;
  }

  void ObjectPropertyGraph::deepCopy(const ObjectPropertyGraph& other)
  {
    for(size_t i = 0; i < other.all_branchs_.size(); i++)
    {
      cpyBranch(other.all_branchs_[i], all_branchs_[i]);
      cpyChainOfBranch(other.all_branchs_[i], all_branchs_[i]);
    }
  }

  void ObjectPropertyGraph::cpyBranch(ObjectPropertyBranch* old_branch, ObjectPropertyBranch* new_branch)
  {
    new_branch->nb_updates_ = old_branch->nb_updates_;
    new_branch->updated_ = old_branch->updated_;
    new_branch->flags_ = old_branch->flags_;

    new_branch->dictionary_ = old_branch->dictionary_;
    new_branch->steady_dictionary_ = old_branch->steady_dictionary_;

    for(const auto& child : old_branch->childs_)
      new_branch->childs_.emplace_back(child, container_.find(child.elem->value()));

    for(const auto& mother : old_branch->mothers_)
      new_branch->mothers_.emplace_back(mother, container_.find(mother.elem->value()));

    for(const auto& range : old_branch->ranges_)
      new_branch->ranges_.emplace_back(range, class_graph_->container_.find(range.elem->value()));

    for(const auto& domain : old_branch->domains_)
      new_branch->domains_.emplace_back(domain, class_graph_->container_.find(domain.elem->value()));

    new_branch->properties_ = old_branch->properties_;

    for(const auto& disjoint : old_branch->disjoints_)
      new_branch->disjoints_.emplace_back(disjoint, container_.find(disjoint.elem->value()));

    for(const auto& inverse : old_branch->inverses_)
      new_branch->inverses_.emplace_back(inverse, container_.find(inverse.elem->value()));

    new_branch->str_chains_ = old_branch->str_chains_;
  }

  void ObjectPropertyGraph::cpyChainOfBranch(ObjectPropertyBranch* old_branch, ObjectPropertyBranch* new_branch)
  {
    for(const auto& chain : old_branch->chains_)
    {
      std::vector<ObjectPropertyBranch*> tmp;
      std::transform(chain.cbegin(), chain.cend(), std::back_inserter(tmp), [this](const auto& link) { return this->container_.find(link->value()); });
      new_branch->chains_.push_back(std::move(tmp));
    }
  }

} // namespace ontologenius
