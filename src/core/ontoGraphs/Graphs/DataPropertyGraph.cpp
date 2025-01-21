#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"

namespace ontologenius {

  DataPropertyGraph::DataPropertyGraph(IndividualGraph* individual_graph,
                                       ClassGraph* class_graph) : OntoGraph(individual_graph),
                                                                  class_graph_(class_graph)
  {}

  DataPropertyGraph::DataPropertyGraph(const DataPropertyGraph& other,
                                       IndividualGraph* individual_graph,
                                       ClassGraph* class_graph) : OntoGraph(individual_graph),
                                                                  class_graph_(class_graph)
  {
    language_ = other.language_;

    for(const auto& branch : other.all_branchs_)
    {
      auto* prop_branch = new DataPropertyBranch(branch->value());
      all_branchs_.push_back(prop_branch);
    }

    this->container_.load(all_branchs_);
  }

  DataPropertyBranch* DataPropertyGraph::add(const std::string& value, DataPropertyVectors_t& property_vectors)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<DataPropertyBranch>::mutex_);
    /**********************
    ** Mothers
    **********************/
    DataPropertyBranch* me = findOrCreateBranch(value);

    // for all my mothers
    for(auto& mother : property_vectors.mothers_)
    {
      DataPropertyBranch* mother_branch = findOrCreateBranch(mother.elem);

      conditionalPushBack(mother_branch->childs_, DataPropertyElement(me, mother.probability, true));
      conditionalPushBack(me->mothers_, DataPropertyElement(mother_branch, mother.probability));
    }

    /**********************
    ** Disjoints
    **********************/
    // for all my disjoints
    for(auto& disjoint : property_vectors.disjoints_)
    {
      DataPropertyBranch* disjoint_branch = findOrCreateBranch(disjoint.elem);

      conditionalPushBack(me->disjoints_, DataPropertyElement(disjoint_branch, disjoint.probability));
      conditionalPushBack(disjoint_branch->disjoints_, DataPropertyElement(me, disjoint.probability, true));
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
    for(const auto& range : property_vectors.ranges_)
    {
      LiteralNode* literal = createLiteralUnsafe(range + "#");
      conditionalPushBack(me->ranges_, literal);
    }

    /**********************
    ** Language and properties
    **********************/
    me->properties_.apply(property_vectors.properties_);
    me->annotation_usage_ = me->annotation_usage_ || property_vectors.annotation_usage_;
    me->setSteadyDictionary(property_vectors.dictionary_);
    me->setSteadyMutedDictionary(property_vectors.muted_dictionary_);

    mitigate(me);
    return me;
  }

  void DataPropertyGraph::add(std::vector<std::string>& disjoints)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<DataPropertyBranch>::mutex_);

    for(size_t disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
    {
      // I need to find myself
      DataPropertyBranch* me = findOrCreateBranch(disjoints[disjoints_i]);

      // for all my disjoints ...
      for(size_t disjoints_j = 0; disjoints_j < disjoints.size(); disjoints_j++)
      {
        //... excepted me
        if(disjoints_i != disjoints_j)
        {
          DataPropertyBranch* disjoint_branch = findOrCreateBranch(disjoints[disjoints_j]);

          conditionalPushBack(me->disjoints_, DataPropertyElement(disjoint_branch));
          conditionalPushBack(disjoint_branch->disjoints_, DataPropertyElement(me, 1.0, true));
        }
      }
    }
  }

  bool DataPropertyGraph::addAnnotation(const std::string& value, DataPropertyVectors_t& property_vectors)
  {
    /**********************
    ** Mothers
    **********************/
    DataPropertyBranch* me = this->container_.find(value);
    if(me == nullptr)
    {
      DataPropertyBranch* mother_branch = nullptr;
      for(auto& mother : property_vectors.mothers_)
      {
        mother_branch = this->container_.find(mother.elem);
        if(mother_branch != nullptr)
          break;
      }

      // I do not exist but one of my mother do so I should exist
      if(mother_branch != nullptr)
      {
        add(value, property_vectors);
        return true;
      }
      else
      {
        ClassBranch* range_branch = nullptr;
        for(auto& range : property_vectors.ranges_)
        {
          range_branch = class_graph_->container_.find(range);
          if(range_branch != nullptr)
            break;
        }

        // My ranges are not classes so there are data and I should exists
        if((range_branch == nullptr) && (property_vectors.ranges_.empty() == false))
        {
          add(value, property_vectors);
          return true;
        }
        else
          return false;
      }
    }
    else
    {
      add(value, property_vectors);
      return true;
    }
  }

  LiteralNode* DataPropertyGraph::createLiteral(const std::string& value)
  {
    LiteralNode* literal = nullptr;
    {
      const std::shared_lock<std::shared_timed_mutex> lock(mutex_);
      literal = literal_container_.find(value);
    }

    if(literal == nullptr)
    {
      const std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      literal = new LiteralNode(value);
      literal_container_.insert(literal);
    }
    return literal;
  }

  LiteralNode* DataPropertyGraph::createLiteralUnsafe(const std::string& value)
  {
    LiteralNode* literal = nullptr;
    literal = literal_container_.find(value);

    if(literal == nullptr)
    {
      literal = new LiteralNode(value);
      literal_container_.insert(literal);
    }
    return literal;
  }

  std::unordered_set<std::string> DataPropertyGraph::getDomain(const std::string& value, size_t depth)
  {
    std::unordered_set<std::string> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch>::mutex_);

    DataPropertyBranch* branch = container_.find(value);
    std::unordered_set<DataPropertyBranch*> up_trace;
    if(branch != nullptr)
      getDomain(branch, depth, res, up_trace);

    return res;
  }

  std::unordered_set<index_t> DataPropertyGraph::getDomain(index_t value, size_t depth)
  {
    std::unordered_set<index_t> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch>::mutex_);

    DataPropertyBranch* branch = container_.find(ValuedNode::table.get(value));
    std::unordered_set<DataPropertyBranch*> up_trace;
    if(branch != nullptr)
      getDomain(branch, depth, res, up_trace);

    return res;
  }

  void DataPropertyGraph::getDomainPtr(DataPropertyBranch* branch, std::unordered_set<ClassBranch*>& res, size_t depth)
  {
    std::unordered_set<DataPropertyBranch*> up_trace;
    if(branch != nullptr)
      getDomainPtr(branch, depth, res, up_trace);
  }

  template<typename T>
  void DataPropertyGraph::getDomain(DataPropertyBranch* branch, size_t depth, std::unordered_set<T>& res, std::unordered_set<DataPropertyBranch*>& up_trace)
  {
    for(auto& domain : branch->domains_)
      class_graph_->getDown(domain.elem, res, depth);

    for(auto& mother : branch->mothers_)
      if(up_trace.insert(mother.elem).second)
        getDomain(mother.elem, depth, res, up_trace);
  }

  void DataPropertyGraph::getDomainPtr(DataPropertyBranch* branch, size_t depth, std::unordered_set<ClassBranch*>& res, std::unordered_set<DataPropertyBranch*>& up_trace)
  {
    for(auto& domain : branch->domains_)
      class_graph_->getDownPtr(domain.elem, res, (int)depth);

    for(auto& mother : branch->mothers_)
      if(up_trace.insert(mother.elem).second)
        getDomainPtr(mother.elem, depth, res, up_trace);
  }

  std::unordered_set<std::string> DataPropertyGraph::getRange(const std::string& value)
  {
    std::unordered_set<std::string> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch>::mutex_);

    DataPropertyBranch* branch = container_.find(value);
    if(branch != nullptr)
      for(auto& range : branch->ranges_)
        res.insert(range->type_);

    return res;
  }

  std::unordered_set<index_t> DataPropertyGraph::getRange(index_t value)
  {
    std::unordered_set<index_t> res;
    const std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch>::mutex_);

    DataPropertyBranch* branch = container_.find(ValuedNode::table.get(value));
    if(branch != nullptr)
      for(auto& range : branch->ranges_)
        res.insert(range->get());

    return res;
  }

  index_t DataPropertyGraph::getLiteralIndex(const std::string& name)
  {
    const std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    auto* branch = literal_container_.find(name);
    if(branch != nullptr)
      return branch->get();
    else
      return 0;
  }

  std::vector<index_t> DataPropertyGraph::getLiteralIndexes(const std::vector<std::string>& names)
  {
    std::vector<index_t> res;
    std::transform(names.cbegin(), names.cend(), std::back_inserter(res), [this](const auto& name) { return getLiteralIndex(name); });
    return res;
  }

  std::string DataPropertyGraph::getLiteralIdentifier(index_t index)
  {
    const std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    if((index < 0) && (index > (index_t)LiteralNode::table.size()))
      return LiteralNode::table[-index];
    else
      return "";
  }

  std::vector<std::string> DataPropertyGraph::getLiteralIdentifiers(const std::vector<index_t>& indexes)
  {
    std::vector<std::string> res;
    std::transform(indexes.cbegin(), indexes.cend(), std::back_inserter(res), [this](const auto& index) { return getLiteralIdentifier(index); });
    return res;
  }

  void DataPropertyGraph::deepCopy(const DataPropertyGraph& other)
  {
    for(size_t i = 0; i < other.all_branchs_.size(); i++)
      cpyBranch(other.all_branchs_[i], all_branchs_[i]);
  }

  void DataPropertyGraph::cpyBranch(DataPropertyBranch* old_branch, DataPropertyBranch* new_branch)
  {
    new_branch->nb_updates_ = old_branch->nb_updates_;
    new_branch->setUpdated(old_branch->isUpdated());
    new_branch->flags_ = old_branch->flags_;

    new_branch->dictionary_ = old_branch->dictionary_;
    new_branch->steady_dictionary_ = old_branch->steady_dictionary_;

    for(const auto& child : old_branch->childs_)
      new_branch->childs_.emplace_back(child, container_.find(child.elem->value()));

    for(const auto& mother : old_branch->mothers_)
      new_branch->mothers_.emplaceBack(mother, container_.find(mother.elem->value()));

    new_branch->ranges_ = old_branch->ranges_;

    for(const auto& domain : old_branch->domains_)
      new_branch->domains_.emplace_back(domain, class_graph_->container_.find(domain.elem->value()));

    new_branch->properties_ = old_branch->properties_;

    for(const auto& disjoint : old_branch->disjoints_)
      new_branch->disjoints_.emplace_back(disjoint, container_.find(disjoint.elem->value()));
  }

} // namespace ontologenius
