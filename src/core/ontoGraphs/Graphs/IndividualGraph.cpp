#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include <algorithm>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

IndividualGraph::IndividualGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

IndividualGraph::IndividualGraph(const IndividualGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;

  language_ = other.language_;

  std::transform(other.all_branchs_.cbegin(), other.all_branchs_.cend(), std::back_inserter(all_branchs_), [](auto indiv){ return new IndividualBranch_t(indiv->value()); });
  for(auto individual : all_branchs_)
  {
    if((size_t)individual->get() >= ordered_individuals_.size())
      ordered_individuals_.resize(individual->get() + 1, nullptr);
    ordered_individuals_[individual->get()] = individual;
  }

  container_.load(all_branchs_);
}

IndividualBranch_t* IndividualGraph::add(const std::string& value, IndividualVectors_t& individual_vector)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  //am I created ?
  IndividualBranch_t* me = container_.find(value);
  bool is_new = false;
  if(me == nullptr)
  {
    me = new IndividualBranch_t(value);
    insertBranchInVectors(me);
    container_.insert(me);
    is_new = true;
  }

  /**********************
  ** Class assertion
  **********************/
  //for all my classes
  for(auto& is_a : individual_vector.is_a_)
  {
    ClassBranch_t* mother_branch = class_graph_->findOrCreateBranch(is_a.elem);

    if(is_new)
    {
      me->is_a_.emplace_back(mother_branch);
      mother_branch->individual_childs_.emplace_back(me);
    }
    else
    {
      if(conditionalPushBack(me->is_a_, ClassElement_t(mother_branch)))
        mother_branch->individual_childs_.emplace_back(me);
    }
  }

  /**********************
  ** Object Property assertion
  **********************/
  for(auto& object_relation : individual_vector.object_relations_)
    addObjectRelation(me, object_relation);

  /**********************
  ** Data Property assertion name
  **********************/
  //for all my properties
  for(auto& data_relation : individual_vector.data_relations_)
    addDataRelation(me, data_relation);

  /**********************
  ** Same In Individual
  **********************/
  addSames(me, individual_vector.same_as_);

  me->setSteadyDictionary(individual_vector.dictionary_);
  me->setSteadyMutedDictionary(individual_vector.muted_dictionary_);

  return me;
}

void IndividualGraph::add(std::vector<std::string>& distinct)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  for(size_t distinct_i = 0; distinct_i < distinct.size(); distinct_i++)
  {
    //am I created ?
    IndividualBranch_t* me = findOrCreateBranch(distinct[distinct_i]);

    //for all my distincts ...
    for(size_t distinct_j = 0; distinct_j < distinct.size(); distinct_j++)
    {
      //... excepted me
      if(distinct_i != distinct_j)
      {
        bool i_find_my_distinct = false;

        //is my distinct created ?
        for(auto& individual : all_branchs_)
          if(distinct[distinct_j] == individual->value())
          {
            conditionalPushBack(me->distinct_, IndividualElement_t(individual));
            i_find_my_distinct = true;
            break;
          }

        //I create my distinct
        if(!i_find_my_distinct)
        {
          auto my_distinct = new IndividualBranch_t(distinct[distinct_j]);
          conditionalPushBack(me->distinct_, IndividualElement_t(my_distinct));
          insertBranchInVectors(my_distinct);
          container_.insert(my_distinct);
        }
      }
    }
  }
}

/*********
*
* add functions
*
*********/

void IndividualGraph::addSames(IndividualBranch_t* me, const std::vector<Single_t<std::string>>& sames, bool is_new)
{
  for(auto& same_as : sames)
  {
    //is my same already created ?
    IndividualBranch_t* same = container_.find(same_as.elem);
    if(same != nullptr)
    {
      for(auto& my_other_same : same->same_as_)
      {
        if(conditionalPushBack(me->same_as_, IndividualElement_t(my_other_same.elem, 1.0, true)))
          my_other_same.elem->same_as_.emplace_back(me, 1.0, true);
      }

      if(is_new == false)
      {
        for(auto& my_other_same : me->same_as_)
        {
          if(conditionalPushBack(same->same_as_, IndividualElement_t(my_other_same.elem, 1.0, true)))
            my_other_same.elem->same_as_.emplace_back(same, 1.0, true);
        }
      }

      if(conditionalPushBack(me->same_as_, IndividualElement_t(same)))
        same->same_as_.emplace_back(me, 1.0, true);
      
      conditionalPushBack(same->same_as_, IndividualElement_t(same, 1.0, true));
    }
    else
    {
      //I create my same
      auto my_same = new IndividualBranch_t(same_as.elem);

      if(is_new == false)
      {
        // my same is new but I'm not
        for(auto& my_other_same : me->same_as_)
        {
          my_other_same.elem->same_as_.emplace_back(my_same, 1.0, true);
          my_same->same_as_.emplace_back(my_other_same.elem, 1.0, true);
        }
      }

      me->same_as_.emplace_back(my_same);
      my_same->same_as_.emplace_back(me, 1.0, true);
      my_same->same_as_.emplace_back(my_same, 1.0, true);
      insertBranchInVectors(my_same);
      container_.insert(my_same);
    }
  }

  if(me->same_as_.size())
    conditionalPushBack(me->same_as_, IndividualElement_t(me, 1.0, true));
}

void IndividualGraph::addObjectRelation(IndividualBranch_t* me, Pair_t<std::string, std::string>& relation)
{
  ObjectPropertyBranch_t* property_branch = object_property_graph_->findOrCreateBranch(relation.first);
  IndividualBranch_t* indiv_branch = findOrCreateBranch(relation.second);

  me->object_relations_.emplace_back(property_branch, indiv_branch, relation.probability);
}

void IndividualGraph::addDataRelation(IndividualBranch_t* me, Pair_t<std::string, std::string>& relation)
{
  DataPropertyBranch_t* property_branch = data_property_graph_->findOrCreateBranch(relation.first);
  LiteralNode* literal = data_property_graph_->createLiteral(relation.second);
  
  me->data_relations_.emplace_back(property_branch, literal, relation.probability);
}

/*********
*
* get functions
*
*********/

std::unordered_set<std::string> IndividualGraph::getSame(const std::string& individual)
{
  return getSame(container_.find(individual));
}

std::unordered_set<index_t> IndividualGraph::getSame(index_t individual)
{
  return getSameId(ordered_individuals_[individual]);
}

std::unordered_set<std::string> IndividualGraph::getDistincts(const std::string& individual)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(individual);
  return getDistincts<std::string>(indiv);
}

std::unordered_set<index_t> IndividualGraph::getDistincts(index_t individual)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = ordered_individuals_[individual];
  return getDistincts<index_t>(indiv);
}

template<typename T>
std::unordered_set<T> IndividualGraph::getDistincts(IndividualBranch_t* individual)
{
  std::unordered_set<T> res;
  if(individual != nullptr)
    for(auto& distinct : individual->distinct_)
      getSame(distinct.elem, res);
  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelationFrom(const std::string& individual, int depth)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(individual);
  return getRelationFrom<std::string>(indiv, depth);
}

std::unordered_set<index_t> IndividualGraph::getRelationFrom(index_t individual, int depth)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = ordered_individuals_[individual];
  return getRelationFrom<index_t>(indiv, depth);
}

template<typename T>
std::unordered_set<T> IndividualGraph::getRelationFrom(IndividualBranch_t* individual, int depth)
{
  std::unordered_set<T> res;
  if(individual != nullptr)
  {
    std::unordered_set<IndividualBranch_t*> sames;
    getSame(individual, sames);
    for(IndividualBranch_t* it : sames)
    {
      for(IndivObjectRelationElement_t& relation : it->object_relations_)
        object_property_graph_->getUp(relation.first, res, depth);

      for(IndivDataRelationElement_t& relation : it->data_relations_)
        data_property_graph_->getUp(relation.first, res, depth);

      std::unordered_set<ClassBranch_t*> up_set;
      getUpPtr(it, up_set);
      for(auto up : up_set)
        getRelationFrom(up, res, depth);
    }
  }
  return res;
}

template<typename T>
void IndividualGraph::getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<T>& res, int depth)
{
  if(class_branch != nullptr)
  {
    for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
      object_property_graph_->getUp(relation.first, res, depth);

    for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
      data_property_graph_->getUp(relation.first, res, depth);
  }
}

std::unordered_set<std::string> IndividualGraph::getRelatedFrom(const std::string& property)
{
  return getRelatedFrom<std::string>(property);
}

std::unordered_set<index_t> IndividualGraph::getRelatedFrom(index_t property)
{
  return getRelatedFrom<index_t>(property);
}

template<typename T>
std::unordered_set<T> IndividualGraph::getRelatedFrom(const T& property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<T> class_res;
  class_graph_->getRelatedFrom(object_properties, data_properties, class_res);

  std::unordered_set<T> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(auto& individual : all_branchs_)
  {
    for(IndivObjectRelationElement_t& relation : individual->object_relations_)
      if(std::any_of(object_properties.begin(), object_properties.end(), [prop_id = relation.first->get()](auto& id){ return id == prop_id; }))
        getSame(individual, res);
    
    for(IndivDataRelationElement_t& relation : individual->data_relations_)
      if(std::any_of(object_properties.begin(), object_properties.end(), [prop_id = relation.first->get()](auto& id){ return id == prop_id; }))
        getSame(individual, res);

    std::unordered_set<T> up_set;
    getUp(individual, up_set, 1);
    for(auto& up : up_set)
      if(class_res.find(up) != class_res.end())
      {
        getSame(individual, res);
        break;
      }
  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelationOn(const std::string& individual, int depth)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  std::unordered_set<index_t> same = getSameId(individual);
  for(index_t id : same)
    for(auto& indiv : all_branchs_)
      for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
        if(relation.second->get() == id)
          object_property_graph_->getUpSafe(relation.first, res, depth);

  if(res.size() == 0)
  {
    LiteralNode* literal = data_property_graph_->literal_container_.find(individual);

    if(literal != nullptr)
      for(auto& indiv : all_branchs_)
        for(IndivDataRelationElement_t& relation : indiv->data_relations_)
          if(relation.second == literal)
            data_property_graph_->getUpSafe(relation.first, res, depth);

    class_graph_->getRelationOnDataProperties(individual, res, depth);
  }

  return res;
}

std::unordered_set<index_t> IndividualGraph::getRelationOn(index_t individual, int depth)
{
  std::unordered_set<index_t> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  if(individual > 0)
  {
    std::unordered_set<index_t> same = getSameId(individual);
    for(index_t id : same)
      for(auto& indiv : all_branchs_)
        for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
          if(relation.second->get() == id)
            object_property_graph_->getUpSafe(relation.first, res, depth);
  }
  else
  {
    for(auto& indiv : all_branchs_)
      for(IndivDataRelationElement_t& relation : indiv->data_relations_)
        if(relation.second->get() == individual)
          data_property_graph_->getUpSafe(relation.first, res, depth);

    class_graph_->getRelationOnDataProperties(LiteralNode::table_.get(-individual), res, depth);
  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelatedOn(const std::string& property)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  getRelatedOn(property, res);

  return res;
}

std::unordered_set<index_t> IndividualGraph::getRelatedOn(index_t property)
{
  std::unordered_set<index_t> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  getRelatedOn(property, res);

  return res;
}

template<typename T>
void IndividualGraph::getRelatedOn(const T& property, std::unordered_set<T>& res)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);
  
  for(auto& indiv : all_branchs_)
  {
    for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
      for (index_t id : object_properties)
        if(relation.first->get() == id)
          getSame(relation.second, res);

    for(IndivDataRelationElement_t& relation : indiv->data_relations_)
      for (index_t id : data_properties)
        if(relation.first->get() == id)
          insert(res, relation.second);
  }

  class_graph_->getRelatedOnDataProperties(property, res);
}

std::unordered_set<std::string> IndividualGraph::getRelationWith(const std::string& individual)
{
  std::unordered_set<std::string> res;

  std::map<index_t, int> properties;
  std::vector<int> depths;
  std::vector<std::string> tmp_res;

  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  IndividualBranch_t* indiv = container_.find(individual);
  if(indiv != nullptr)
  {
    std::unordered_set<IndividualBranch_t*> sames;
    getSame(indiv, sames);
    for(IndividualBranch_t* it : sames)
    {
      for(IndivObjectRelationElement_t& relation : it->object_relations_)
      {
        getSame(relation.second, res);

        properties[relation.first->get()] = tmp_res.size();
        depths.push_back(0);
        tmp_res.push_back(relation.second->value());
      }

      for(IndivDataRelationElement_t& relation : it->data_relations_)
      {
        res.insert(relation.second->value());

        properties[relation.first->get()] = tmp_res.size();
        depths.push_back(0);
        tmp_res.push_back(relation.second->value());
      }
    }

    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(indiv, up_set, 1);
    for(auto up : up_set)
      class_graph_->getRelationWith(up, properties, depths, tmp_res, 0);
    for(auto& it : tmp_res)
      res.insert(it);
  }
  return res;
}

std::unordered_set<index_t> IndividualGraph::getRelationWith(index_t individual)
{
  std::unordered_set<index_t> res;

  std::map<index_t, int> properties;
  std::vector<int> depths;
  std::vector<index_t> tmp_res;

  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  IndividualBranch_t* indiv = ordered_individuals_[individual];
  if(indiv != nullptr)
  {
    std::unordered_set<IndividualBranch_t*> sames;
    getSame(indiv, sames);
    for(IndividualBranch_t* it : sames)
    {
      for(IndivObjectRelationElement_t& relation : it->object_relations_)
      {
        getSame(relation.second, res);

        properties[relation.first->get()] = tmp_res.size();
        depths.push_back(0);
        tmp_res.push_back(relation.second->get());
      }

      for(IndivDataRelationElement_t& relation : it->data_relations_)
      {
        res.insert(relation.second->get());

        properties[relation.first->get()] = tmp_res.size();
        depths.push_back(0);
        tmp_res.push_back(relation.second->get());
      }
    }

    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(indiv, up_set, 1);
    for(auto up : up_set)
      class_graph_->getRelationWith(up, properties, depths, tmp_res, 0);
    for(auto& it : tmp_res)
      res.insert(it);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelatedWith(const std::string& individual)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  index_t indiv_index = 0;
  auto indiv_ptr = container_.find(individual);
  if(indiv_ptr != nullptr)
    indiv_index = indiv_ptr->get();
  else
  {
    auto literal_ptr = data_property_graph_->literal_container_.find(individual);
    if(literal_ptr != nullptr)
      indiv_index = literal_ptr->get();
    else
    {
      auto class_ptr = class_graph_->container_.find(individual);
      if(class_ptr != nullptr)
        indiv_index = class_ptr->get();
    }
  }

  if(indiv_index != 0)
    getRelatedWith(indiv_index, res);

  return res;
}

std::unordered_set<index_t> IndividualGraph::getRelatedWith(index_t individual)
{
  std::unordered_set<index_t> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  getRelatedWith(individual, res);

  return res;
}

template<typename T>
void IndividualGraph::getRelatedWith(index_t individual, std::unordered_set<T>& res)
{
  for(auto& indiv : all_branchs_)
  {
    bool found = false;
    std::unordered_set<index_t> took;

    if(individual > 0)
    {
      for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
        if(relation.second->get() == individual)
        {
          found = true;
          took.insert(relation.first->get());
        }
    }
    else
    {
      for(IndivDataRelationElement_t& relation : indiv->data_relations_)
      {
        if(relation.second->get() == individual)
        {
          found = true;
          took.insert(relation.first->get());
        }
      }
    }

    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(indiv, up_set, 1);
    while(up_set.size() > 0)
    {
      std::unordered_set<ClassBranch_t*> next_step;
      for(auto up : up_set)
        found = found || getRelatedWith(up, individual, next_step, took);

      up_set = next_step;
    }

    if(found == true)
      getSame(indiv, res);
  }
}

bool IndividualGraph::getRelatedWith(ClassBranch_t* class_branch, index_t data, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<index_t>& took)
{
  bool res = false;
  if(class_branch != nullptr)
  {
    if(data > 0)
    {
      for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
      {
        if(relation.second->get() == data)
          if(took.find(relation.first->get()) == took.end())
            res = true;
        took.insert(relation.first->get());
      }
    }
    else
    {
      for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
      {
        if(relation.second->get() == data)
          if(took.find(relation.first->get()) == took.end())
            res = true;
        took.insert(relation.first->get());
      }
    }

    class_graph_->getUpPtr(class_branch, next_step, 1);
    next_step.erase(class_branch);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getFrom(const std::string& param)
{
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    std::string individual = param.substr(0, pose);
    std::string property = param.substr(pose+1);
    return getFrom(individual, property);
  }
  return {};
}

std::unordered_set<std::string> IndividualGraph::getFrom(const std::string& individual, const std::string& property, bool single_same)
{
  std::unordered_set<std::string> res;
  index_t indiv_index = 0;
  auto indiv_ptr = container_.find(individual);
  if(indiv_ptr != nullptr)
    indiv_index = indiv_ptr->get();
  else
  {
    auto literal_ptr = data_property_graph_->literal_container_.find(individual);
    if(literal_ptr != nullptr)
      indiv_index = literal_ptr->get();
    else
    {
      auto class_ptr = class_graph_->container_.find(individual);
      if(class_ptr != nullptr)
        indiv_index = class_ptr->get();
    }
  }

  getFrom(indiv_index, property, res, single_same);

  return res;
}

std::unordered_set<index_t> IndividualGraph::getFrom(index_t individual, index_t property, bool single_same)
{
  std::unordered_set<index_t> res;
  getFrom(individual, property, res, single_same);
  return res;
}

template<typename T>
void IndividualGraph::getFrom(index_t individual, const T& property, std::unordered_set<T>& res, bool single_same)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);
  
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  for(auto& indiv_i : all_branchs_)
  {
    bool found = false;

    if(individual > 0)
    {
      for(IndivObjectRelationElement_t& relation : indiv_i->object_relations_)
        for (index_t id : object_properties)
          if(relation.first->get() == id)
          {
            if(relation.second->get() == individual)
            {
              found = true;
              break;
            }
          }
    }
    else if(individual < 0)
      for(IndivDataRelationElement_t& relation : indiv_i->data_relations_)
        for (index_t id : data_properties)
          if(relation.first->get() == id)
          {
            if(relation.second->get() == individual)
            {
              found = true;
              break;
            }
          }

    if(found == false)
    {
      std::unordered_set<index_t> down_classes;
      if(individual > 0)
        down_classes = class_graph_->getDownId(individual);
      std::unordered_set<index_t> do_not_take;

      std::unordered_set<ClassBranch_t*> up_set;
      getUpPtr(indiv_i, up_set, 1);
      while(up_set.size() > 0)
      {
        std::unordered_set<ClassBranch_t*> next_step;
        for(auto up : up_set)
          found = found || getFrom(up, object_properties, data_properties, individual, down_classes, next_step, do_not_take);

        up_set = next_step;
      }
    }

    if(found == true)
    {
      if(single_same)
        getLowestSame(indiv_i, res);
      else
        getSame(indiv_i, res);
    }
  }
}

bool IndividualGraph::getFrom(ClassBranch_t* class_branch, const std::unordered_set<index_t>& object_properties, const std::unordered_set<index_t>& data_properties, index_t data, const std::unordered_set<index_t>& down_classes, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<index_t>& do_not_take)
{
  if(class_branch != nullptr)
  {
    if(do_not_take.find(class_branch->get()) != do_not_take.end())
      return false;

    bool found = false;
    bool defined = false;

    if(down_classes.size())
    {
      for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
        for (index_t id : object_properties)
          if(relation.first->get() == id)
          {
            defined = true;
            if(std::any_of(down_classes.begin(), down_classes.end(), [id = relation.second->get()](auto class_id){ return id == class_id; }))
              found = true;
          }
    }
    else if(data < 0)
      for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
        for (index_t id : data_properties)
          if(relation.first->get() == id)
          {
            defined = true;
            if(relation.second->get() == data)
            {
              found = true;
              break;
            }
          }

    if(defined == true)
    {
      class_graph_->getUpSafe(class_branch, do_not_take);
      return found;
    }
    else
    {
      class_graph_->getUpPtr(class_branch, next_step, 1);
      next_step.erase(class_branch);
    }
  }
  return false;
}

std::unordered_set<std::string> IndividualGraph::getOn(const std::string& param)
{
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    std::string individual = param.substr(0, pose);
    std::string property = param.substr(pose+1);
    return getOn(individual, property);
  }
  return std::unordered_set<std::string>();
}

std::unordered_set<std::string> IndividualGraph::getOn(const std::string& individual, const std::string& property, bool single_same)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(individual);

  return getOn(indiv, property, single_same);
}

std::unordered_set<index_t> IndividualGraph::getOn(index_t individual, index_t property, bool single_same)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = ordered_individuals_[individual];

  return getOn(indiv, property, single_same);
}

// Duplication in this function avoid the creation of the same_as set
// This for performance issues in the SPARQL solver
template<typename T>
std::unordered_set<T> IndividualGraph::getOn(IndividualBranch_t* individual, const T& property, bool single_same)
{
  std::unordered_set<T> res;
  
  if(individual != nullptr)
  {
    std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
    std::unordered_set<index_t> data_properties;
    if(object_properties.size())
    {
      if(individual->same_as_.size())
      {
        for(auto& same_indiv : individual->same_as_)
        {
          for(IndivObjectRelationElement_t& relation : same_indiv.elem->object_relations_)
            for (index_t id : object_properties)
              if(relation.first->get() == id)
              {
                if(single_same)
                  getLowestSame(relation.second, res);
                else
                  getSame(relation.second, res);
              }
        }
      }
      else
      {
        for(IndivObjectRelationElement_t& relation : individual->object_relations_)
          for (index_t id : object_properties)
            if(relation.first->get() == id)
            {
              if(single_same)
                getLowestSame(relation.second, res);
              else
                getSame(relation.second, res);
            }
      }
    }
    else
    {
      data_properties = data_property_graph_->getDownId(property);
      if(data_properties.size())
      {
        if(individual->same_as_.size())
        {
          for(auto& same_indiv : individual->same_as_)
          {
            for(IndivDataRelationElement_t& relation : same_indiv.elem->data_relations_)
              for (index_t id : data_properties)
                if(relation.first->get() == id)
                  insert(res, relation.second);
          }
        }
        else
        {
          for(IndivDataRelationElement_t& relation : individual->data_relations_)
            for (index_t id : data_properties)
              if(relation.first->get() == id)
                insert(res, relation.second);
        }
      }
    }
    
    if(res.size() == 0)
    {
      int found_depth = -1;
      std::unordered_set<ClassBranch_t*> up_set;
      getUpPtr(individual, up_set, 1);
      for(ClassBranch_t* up : up_set)
        class_graph_->getOn(up, object_properties, data_properties, res, 1, found_depth);
    }
  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::getWith(const std::string& param, int depth)
{
  std::unordered_set<std::string> res;
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    std::string first_individual = param.substr(0, pose);
    std::string second_individual = param.substr(pose+1);
    return getWith(first_individual, second_individual, depth);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getWith(const std::string& first_individual, const std::string& second_individual, int depth)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(first_individual);

  std::unordered_set<index_t> second_individual_index;
  auto second_individual_ptr = container_.find(second_individual);
  if(second_individual_ptr != nullptr)
  {
    if(second_individual_ptr->same_as_.empty())
      second_individual_index = {second_individual_ptr->get()};
    else
      second_individual_index = getSameId(second_individual_ptr);
  }
  else
  {
    auto literal = data_property_graph_->literal_container_.find(second_individual);
    if(literal != nullptr)
      second_individual_index = {literal->get()};
    else
    {
      auto second_class_ptr = class_graph_->container_.find(second_individual);
      if(second_class_ptr != nullptr)
        second_individual_index = {second_class_ptr->get()};
    }
  }
  
  getWith(indiv, second_individual_index, res, depth);
  return res;
}

std::unordered_set<index_t> IndividualGraph::getWith(index_t first_individual, index_t second_individual, int depth)
{
  std::unordered_set<index_t> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = ordered_individuals_[first_individual];
  if(second_individual > 0)
  {
    if((size_t)second_individual >= ordered_individuals_.size())
    {
       getWith(indiv, {second_individual}, res, depth); // class
    }
    else
    {
      IndividualBranch_t* second = ordered_individuals_[second_individual];
      if(second == nullptr)
        getWith(indiv, {second_individual}, res, depth); // class
      else if(second->same_as_.empty())
        getWith(indiv, {second_individual}, res, depth);
      else
        getWith(indiv, getSameId(second), res, depth);
    }
  }
  else
    getWith(indiv, {second_individual}, res, depth); // literal
  return res;
}

template<typename T>
void IndividualGraph::getWith(IndividualBranch_t* first_individual, const std::unordered_set<index_t>& second_individual_index, std::unordered_set<T>& res, int depth)
{
  if(first_individual != nullptr)
  {
    if(second_individual_index.empty())
      return;

    if(*second_individual_index.begin() > 0)
    {
      if(first_individual->same_as_.empty())
      {
        for(IndivObjectRelationElement_t& relation : first_individual->object_relations_)
          if(second_individual_index.find(relation.second->get()) != second_individual_index.end())
            object_property_graph_->getUp(relation.first, res, depth);
      }
      else
      {
        for(auto& same : first_individual->same_as_)
          for(IndivObjectRelationElement_t& relation : same.elem->object_relations_)
            if(second_individual_index.find(relation.second->get()) != second_individual_index.end())
              object_property_graph_->getUp(relation.first, res, depth);
      }
    }
    else if(*second_individual_index.begin() < 0)
    {
      if(first_individual->same_as_.empty())
      {
        for(IndivDataRelationElement_t& relation : first_individual->data_relations_)
          if(second_individual_index.find(relation.second->get()) != second_individual_index.end())
            data_property_graph_->getUp(relation.first, res, depth);
      }
      else
      {
        for(auto& same : first_individual->same_as_)
          for(IndivDataRelationElement_t& relation : same.elem->data_relations_)
            if(second_individual_index.find(relation.second->get()) != second_individual_index.end())
              data_property_graph_->getUp(relation.first, res, depth);
      }
    }

    int found_depth = -1;
    uint32_t current_depth = 0;
    std::unordered_set<index_t> do_not_take;
    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(first_individual, up_set, 1);
    while(up_set.size() > 0)
    {
      std::unordered_set<ClassBranch_t*> next_step;
      for(auto up : up_set)
        class_graph_->getWith(up, *second_individual_index.begin(), res, do_not_take, current_depth, found_depth, depth, next_step); // we can take the front of second_individual_index as class does not have same_as

      up_set = next_step;
      current_depth++;
    }
  }
}

std::unordered_set<std::string> IndividualGraph::getDomainOf(const std::string& individual, int depth)
{
  IndividualBranch_t* branch = container_.find(individual);
  std::unordered_set<std::string> res;
  getDomainOf(branch, res, depth);
  return res;
}

std::unordered_set<index_t> IndividualGraph::getDomainOf(index_t individual, int depth)
{
  IndividualBranch_t* branch = ordered_individuals_[individual];
  std::unordered_set<index_t> res;
  getDomainOf(branch, res, depth);
  return res;
}

template<typename T>
void IndividualGraph::getDomainOf(IndividualBranch_t* individual, std::unordered_set<T>& res, int depth)
{
  std::unordered_set<ClassBranch_t*> classes;
  getUpPtr(individual, classes, 1);

  for(auto c : classes)
    class_graph_->getDomainOf(c, res, depth);
}

std::unordered_set<std::string> IndividualGraph::getRangeOf(const std::string& individual, int depth)
{
  IndividualBranch_t* branch = container_.find(individual);
  std::unordered_set<std::string> res;
  getRangeOf(branch, res, depth);
  return res;
}

std::unordered_set<index_t> IndividualGraph::getRangeOf(index_t individual, int depth)
{
  IndividualBranch_t* branch = ordered_individuals_[individual];
  std::unordered_set<index_t> res;
  getRangeOf(branch, res, depth);
  return res;
}

template<typename T>
void IndividualGraph::getRangeOf(IndividualBranch_t* individual, std::unordered_set<T>& res, int depth)
{
  std::unordered_set<ClassBranch_t*> classes;
  getUpPtr(individual, classes, 1);

  for(auto c : classes)
    class_graph_->getRangeOf(c, res, depth);
}

std::unordered_set<std::string> IndividualGraph::getUp(const std::string& individual, int depth)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(individual);
  std::unordered_set<std::string> res;
  getUp(indiv, res, depth);
  return res;
}

std::unordered_set<index_t> IndividualGraph::getUp(index_t individual, int depth)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = ordered_individuals_[individual];
  std::unordered_set<index_t> res;
  getUp(indiv, res, depth);
  return res;
}

template<typename T>
void IndividualGraph::getUp(IndividualBranch_t* indiv, std::unordered_set<T>& res, int depth, uint32_t current_depth)
{
  current_depth++;
  if(indiv != nullptr)
  {
    if(indiv->same_as_.size())
    {
      for(auto& it : indiv->same_as_)
        for(auto& is_a : it.elem->is_a_)
          class_graph_->getUp(is_a.elem, res, depth, current_depth);
    }
    else
    {
      for(auto& is_a : indiv->is_a_)
        class_graph_->getUp(is_a.elem, res, depth, current_depth);
    }
  }
}

void IndividualGraph::getUpPtr(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*>& res, int depth, uint32_t current_depth)
{
  if(indiv != nullptr)
  {
    if(depth != 1)
    {
      current_depth++;
      if(indiv->same_as_.size())
      {
        for(auto& it : indiv->same_as_)
          for(auto& is_a : it.elem->is_a_)
            class_graph_->getUpPtr(is_a.elem, res, depth, current_depth);
      }
      else
      {
        for(auto& is_a : indiv->is_a_)
          class_graph_->getUpPtr(is_a.elem, res, depth, current_depth);
      }
    }
    else
    {
      if(indiv->same_as_.size())
      {
        for(auto& it : indiv->same_as_)
          for(auto& is_a : it.elem->is_a_)
            res.insert(is_a.elem);
      }
      else
      {
        for(auto& is_a : indiv->is_a_)
          res.insert(is_a.elem);
      }
    }
  }
}

void IndividualGraph::getDistincts(IndividualBranch_t* individual, std::unordered_set<IndividualBranch_t*>& res)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  if(individual != nullptr)
  {
    for(auto& distinct : individual->distinct_)
      getSame(distinct.elem, res);
  }
}

std::unordered_set<index_t> IndividualGraph::getSameId(const std::string& individual)
{
  return getSameId(container_.find(individual));
}

std::unordered_set<index_t> IndividualGraph::getSameId(index_t individual)
{
  return getSameId(ordered_individuals_[individual]);
}

void IndividualGraph::getLowestSame(IndividualBranch_t* individual, std::unordered_set<IndividualBranch_t*>& res)
{
  if(individual != nullptr)
  {
    if(individual->same_as_.size())
      res.insert(std::min_element(individual->same_as_.cbegin(), individual->same_as_.cend(),
                 [](auto& a, auto& b){ return a.elem->get() < b.elem->get(); })->elem);
    else
      res.insert(individual);
  }
}

void IndividualGraph::getSame(IndividualBranch_t* individual, std::unordered_set<IndividualBranch_t*>& res)
{
  if(individual != nullptr)
  {
    if(individual->same_as_.size())
      std::transform(individual->same_as_.cbegin(), individual->same_as_.cend(), std::inserter(res, res.begin()), [](const auto& same){ return same.elem; });
    else
      res.insert(individual);
  }
}

void IndividualGraph::getSame(IndividualBranch_t* individual, std::vector<IndividualBranch_t*>& res)
{
  if(individual != nullptr)
  {
    if(individual->same_as_.size())
      std::transform(individual->same_as_.cbegin(), individual->same_as_.cend(), std::back_inserter(res), [](const auto& same){ return same.elem; });
    else
      res.push_back(individual);
  }
}

std::unordered_set<std::string> IndividualGraph::getSame(IndividualBranch_t* individual)
{
  std::unordered_set<std::string> res;
  getSame(individual, res);
  return res;
}

void IndividualGraph::getLowestSame(IndividualBranch_t* individual, std::unordered_set<std::string>& res)
{
  if(individual != nullptr)
  {
    if(individual->same_as_.size())
    {
      auto indiv = std::min_element(individual->same_as_.cbegin(), individual->same_as_.cend(),
                                    [](auto& a, auto& b){ return a.elem->get() < b.elem->get(); });
      res.insert(indiv->elem->value());
    }
    else
      res.insert(individual->value());
  }
}

void IndividualGraph::getSame(IndividualBranch_t* individual, std::unordered_set<std::string>& res)
{
  if(individual != nullptr)
  {
    if(individual->same_as_.size())
      std::transform(individual->same_as_.cbegin(), individual->same_as_.cend(), std::inserter(res, res.begin()), [](const auto& same){ return same.elem->value(); });
    else
      res.insert(individual->value());
  }
}

std::unordered_set<index_t> IndividualGraph::getSameId(IndividualBranch_t* individual)
{
  std::unordered_set<index_t> res;
  getSame(individual, res);
  return res;
}

void IndividualGraph::getLowestSame(IndividualBranch_t* individual, std::unordered_set<index_t>& res)
{
  if(individual != nullptr)
  {
    if(individual->same_as_.size())
    {
      auto indiv = std::min_element(individual->same_as_.cbegin(), individual->same_as_.cend(),
                                    [](auto& a, auto& b){ return a.elem->get() < b.elem->get(); });
      res.insert(indiv->elem->get());
    }
    else
      res.insert(individual->get());
  }
}

void IndividualGraph::getSame(IndividualBranch_t* individual, std::unordered_set<index_t>& res)
{
  if(individual != nullptr)
  {
    if(individual->same_as_.size())
      std::transform(individual->same_as_.cbegin(), individual->same_as_.cend(), std::inserter(res, res.begin()), [](const auto& same){ return same.elem->get(); });
    else
      res.insert(individual->get());
  }
}

std::unordered_set<std::string> IndividualGraph::select(const std::unordered_set<std::string>& on, const std::string& class_selector)
{
  std::unordered_set<std::string> res;
  std::unordered_set<std::string> classes;

  for(const std::string& it : on)
  {
    IndividualBranch_t* branch = container_.find(it);
    if(branch!= nullptr)
    {
      std::unordered_set<std::string> tmp;
      getUp(branch, tmp);
      if(tmp.find(class_selector) != tmp.end())
        res.insert(it);
    }
    else
      classes.insert(it);
  }

  if(classes.size())
  {
    std::unordered_set<std::string> tmp_res = class_graph_->select(classes, class_selector);
    if(tmp_res.size())
      res.insert(tmp_res.begin(), tmp_res.end());
  }

  return res;
}

std::unordered_set<index_t> IndividualGraph::select(const std::unordered_set<index_t>& on, index_t class_selector)
{
  std::unordered_set<index_t> res;
  std::unordered_set<index_t> classes;

  for(index_t it : on)
  {
    if(it > 0)
    {
      IndividualBranch_t* branch = ordered_individuals_[it];;
      if(branch!= nullptr)
      {
        std::unordered_set<index_t> tmp;
        getUp(branch, tmp);
        if(tmp.find(class_selector) != tmp.end())
          res.insert(it);
      }
      else
        classes.insert(it);
    }
  }

  if(classes.size())
  {
    std::unordered_set<index_t> tmp_res = class_graph_->select(classes, class_selector);
    if(tmp_res.size())
      res.insert(tmp_res.begin(), tmp_res.end());
  }

  return res;
}


std::unordered_set<std::string> IndividualGraph::getType(const std::string& class_selector, bool single_same)
{
  std::shared_lock<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

  std::unordered_set<std::string> res;
  ClassBranch_t* class_branch = class_graph_->container_.find(class_selector);
  if(class_branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> down_set = class_graph_->getDownPtrSafe(class_branch);
    for(auto down : down_set)
      class_graph_->getDownIndividual(down, res, single_same);
  }

  return res;
}

std::unordered_set<index_t> IndividualGraph::getType(index_t class_selector, bool single_same)
{
  std::shared_lock<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

  std::unordered_set<index_t> res;
  ClassBranch_t* class_branch = class_graph_->container_.find(ValuedNode::table_.get(class_selector));
  if(class_branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> down_set;
    class_graph_->getDownPtr(class_branch, down_set);
    for(auto down : down_set)
      class_graph_->getDownIndividual(down, res, single_same);
  }

  return res;
}


bool IndividualGraph::isA(const std::string& indiv, const std::string& class_selector)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  std::shared_lock<std::shared_timed_mutex> lock_class(class_graph_->mutex_);
  IndividualBranch_t* branch = container_.find(indiv);
  return isA(branch, class_selector);
}

bool IndividualGraph::isA(index_t indiv, index_t class_selector)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  std::shared_lock<std::shared_timed_mutex> lock_class(class_graph_->mutex_);
  IndividualBranch_t* branch = ordered_individuals_[indiv];
  return isA(branch, class_selector);
}

bool IndividualGraph::isA(IndividualBranch_t* indiv, const std::string& class_selector)
{
  return isATemplate(indiv, class_selector);
}

bool IndividualGraph::isA(IndividualBranch_t* indiv, index_t class_selector)
{
  return isATemplate(indiv, class_selector);
}

template<typename T>
bool IndividualGraph::isATemplate(IndividualBranch_t* branch, const T& class_selector)
{
  if(branch != nullptr)
  {
    if(branch->same_as_.empty())
      return std::any_of(branch->is_a_.cbegin(), branch->is_a_.cend(), [this, class_selector](const auto& is_a){ return class_graph_->existInInheritance(is_a.elem, class_selector); });
    else
    {
      for(auto& it : branch->same_as_)
        if(std::any_of(it.elem->is_a_.cbegin(), it.elem->is_a_.cend(), [this, class_selector](const auto& is_a){ return class_graph_->existInInheritance(is_a.elem, class_selector); }))
          return true;
    }
  }
  return false;
}

bool IndividualGraph::relationExists(const std::string& param)
{
  std::string subject;
  std::string property;
  std::string object;
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    subject = param.substr(0, pose);
    property = param.substr(pose+1);
    pose = property.find(':');
    if(pose != std::string::npos)
    {
      object = property.substr(pose+1);
      property = property.substr(0, pose);
      return relationExists(subject, property, object);
    }
    else
      return false;
  }
  return false;
}

bool IndividualGraph::relationExists(const std::string& subject, const std::string& property, const std::string& object)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* subject_branch = container_.find(subject);
  if(subject_branch == nullptr)
    return false;

  std::unordered_set<IndividualBranch_t*> sames;
  getSame(subject_branch, sames);
  for(IndividualBranch_t* it : sames)
  {
    for(IndivObjectRelationElement_t& relation : it->object_relations_)
    {
      if(relation.first->value() != property)
        continue;
      else
      {
        std::unordered_set<IndividualBranch_t*> sames_tmp;
        getSame(relation.second, sames_tmp);
        if(std::any_of(sames_tmp.begin(), sames_tmp.end(), [object](const auto& same){ return object == same->value(); }))
          return true;
      }
    }

    for(IndivDataRelationElement_t& relation : it->data_relations_)
    {
      if(relation.first->value() != property)
        continue;
      else if(relation.second->value() == object)
        return true;
    }
  }

  return false;
}

bool IndividualGraph::relationExists(IndividualBranch_t* subject, ObjectPropertyBranch_t* property, IndividualBranch_t* object)
{
  std::unordered_set<IndividualBranch_t*> sames;
  getSame(subject, sames);
  for(IndividualBranch_t* it : sames)
  {
    for(IndivObjectRelationElement_t& relation : it->object_relations_)
    {
      if(relation.first != property)
        continue;
      else
      {
        std::unordered_set<IndividualBranch_t*> sames_tmp;
        getSame(relation.second, sames_tmp);
        if(std::any_of(sames_tmp.begin(), sames_tmp.end(), [object](auto& same){ return object == same; }))
          return true;
      }
    }
  }

  return false;
}

ClassBranch_t* IndividualGraph::upgradeToBranch(IndividualBranch_t* indiv)
{
  if(indiv != nullptr)
  {
    auto class_branch = new ClassBranch_t(indiv->value());
    class_branch->mothers_.relations = std::move(indiv->is_a_.relations);
    class_branch->data_relations_.clear();
    for(auto& data_relation : indiv->data_relations_)
      class_branch->data_relations_.emplace_back(data_relation.first,data_relation.second, data_relation.probability);
    class_branch->dictionary_ = std::move(indiv->dictionary_);
    class_branch->steady_dictionary_ = std::move(indiv->steady_dictionary_);

    class_graph_->container_.insert(class_branch);
    class_graph_->all_branchs_.push_back(class_branch);
    redirectDeleteIndividual(indiv, class_branch);

    return class_branch;
  }
  return nullptr;
}

IndividualBranch_t* IndividualGraph::findOrCreateBranchSafe(const std::string& name)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  return findOrCreateBranch(name);
}

IndividualBranch_t* IndividualGraph::findOrCreateBranch(const std::string& name)
{
  IndividualBranch_t* indiv = Graph::findOrCreateBranch(name);
  if(indiv != nullptr)
  {
    if((size_t)indiv->get() >= ordered_individuals_.size())
      ordered_individuals_.resize(indiv->get() + 1, nullptr);
    ordered_individuals_[indiv->get()] = indiv;
  }
  return indiv;
}

void IndividualGraph::deleteIndividual(IndividualBranch_t* indiv)
{
  if(indiv != nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);

    // erase indiv from same_as
    for(auto& same : indiv->same_as_)
      if(same.elem != indiv)
        removeFromElemVect(same.elem->same_as_, indiv);
    indiv->same_as_.clear();

    // erase indiv from parents
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);
    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(indiv, up_set, 1);

    for(auto up : up_set)
    {
      for(size_t i = 0; i < up->individual_childs_.size();)
      {
        if(up->individual_childs_[i] == indiv)
          up->individual_childs_.erase(up->individual_childs_.begin() + i);
        else
          i++;
      }
    }

    // erase relations applied toward indiv
    size_t indiv_index = 0;
    for(size_t indiv_i = 0; indiv_i < all_branchs_.size(); indiv_i++)
    {
      if(all_branchs_[indiv_i] == indiv)
        indiv_index = indiv_i;

      for(size_t i = 0; i < all_branchs_[indiv_i]->object_relations_.size();)
        if(all_branchs_[indiv_i]->object_relations_[i].second == indiv)
          all_branchs_[indiv_i]->object_relations_.erase(i);
        else
          i++;
    }

    //delete indiv
    removeBranchInVectors(indiv_index);
    container_.erase(indiv);
    delete indiv;
  }
}

void IndividualGraph::redirectDeleteIndividual(IndividualBranch_t* indiv, ClassBranch_t* _class)
{
  if(indiv != nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

    // erase indiv from parents
    std::unordered_set<ClassBranch_t*> up_set;
    class_graph_->getUpPtr(_class, up_set, 1);

    for(auto up : up_set)
    {
      for(size_t i = 0; i < up->individual_childs_.size();)
      {
        if(up->individual_childs_[i] == indiv)
        {
          up->individual_childs_.erase(up->individual_childs_.begin() + i);
          up->childs_.emplace_back(_class);
        }
        else
          i++;
      }
    }

    //erase properties applied to indiv
    size_t indiv_index = 0;
    for(size_t indiv_i = 0; indiv_i < all_branchs_.size(); indiv_i++)
    {
      if(all_branchs_[indiv_i] == indiv)
        indiv_index = indiv_i;

      for(size_t i = 0; i < all_branchs_[indiv_i]->object_relations_.size();)
        if(all_branchs_[indiv_i]->object_relations_[i].second == indiv)
          all_branchs_[indiv_i]->object_relations_.erase(i);
        else
          i++;
    }

    //delete indiv
    removeBranchInVectors(indiv_index);
    container_.erase(indiv);
    delete indiv;
  }
}

bool IndividualGraph::addInheritage(const std::string& indiv, const std::string& class_inherited)
{
  IndividualBranch_t* branch = findBranchSafe(indiv);
  return addInheritage(branch, class_inherited);
}

bool IndividualGraph::addInheritage(IndividualBranch_t* branch, const std::string& class_inherited)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);
  return addInheritageUnsafe(branch, class_inherited);
}

bool IndividualGraph::addInheritageUnsafe(IndividualBranch_t* branch, const std::string& class_inherited)
{
  if(branch != nullptr)
  {
    ClassBranch_t* inherited = class_graph_->findBranch(class_inherited);
    if(inherited == nullptr)
    {
      IndividualBranch_t* tmp = findBranch(class_inherited);
      if(tmp != nullptr)
        inherited = upgradeToBranch(tmp);
      else
      {
        inherited = new ClassBranch_t(class_inherited);
        class_graph_->container_.insert(inherited);
        class_graph_->all_branchs_.push_back(inherited);
      }
    }
    conditionalPushBack(branch->is_a_, ClassElement_t(inherited));
    conditionalPushBack(inherited->individual_childs_, IndividualElement_t(branch));
    branch->updated_ = true;
    inherited->updated_ = true;

    return true; // TODO verify that multi inheritances are compatible
  }
  else
    return false;
}

bool IndividualGraph::addInheritageInvert(const std::string& indiv, const std::string& class_inherited)
{
  ClassBranch_t* inherited = class_graph_->findBranchSafe(class_inherited);
  if(inherited != nullptr)
  {
    IndividualBranch_t* branch = findOrCreateBranchSafe(indiv);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

    conditionalPushBack(branch->is_a_, ClassElement_t(inherited));
    conditionalPushBack(inherited->individual_childs_, IndividualElement_t(branch));
    branch->updated_ = true;
    inherited->updated_ = true;

    return true; // TODO verify that multi inheritances are compatible
  }
  else
    return false;
}

bool IndividualGraph::addInheritageInvertUpgrade(const std::string& indiv, const std::string& class_inherited)
{
  IndividualBranch_t* tmp = findBranchSafe(class_inherited);
  if(tmp != nullptr)
  {
    ClassBranch_t* inherited = upgradeToBranch(tmp);
    IndividualBranch_t* branch = findOrCreateBranchSafe(indiv);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

    conditionalPushBack(branch->is_a_, ClassElement_t(inherited));
    conditionalPushBack(inherited->individual_childs_, IndividualElement_t(branch));
    branch->updated_ = true;
    inherited->updated_ = true;

    return true; // TODO verify that multi inheritances are compatible
  }
  else
    return false;
}

int IndividualGraph::addRelation(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on, double proba, bool infered, bool check_existance)
{
  if(object_property_graph_->isIrreflexive(property))
  {
    auto ids = getSameId(indiv_from);
    if(ids.find(indiv_on->get()) != ids.end())
      throw GraphException("Inconsistency prevented regarding irreflexivity of the property");
  }

  if(object_property_graph_->isAsymetric(property))
  {
    if(relationExists(indiv_on, property, indiv_from))
      throw GraphException("Inconsistency prevented regarding asymetry of the property");
  }

  int index = -1;
  if(check_existance)
    index = indiv_from->objectRelationExists(property, indiv_on);
  if(index == -1)
  {
    if(checkRangeAndDomain(indiv_from, property, indiv_on) == false)
      throw GraphException("Inconsistency prevented regarding the range or domain of the property");

    indiv_from->object_relations_.emplace_back(property, indiv_on);
    index = indiv_from->object_relations_.size() - 1;
    indiv_on->updated_ = true;
    indiv_from->updated_ = true;
  }

  indiv_from->object_relations_[index].probability = proba;
  indiv_from->object_relations_[index].infered = infered;

  return index;
}

int IndividualGraph::addRelation(IndividualBranch_t* indiv_from, DataPropertyBranch_t* property, LiteralNode* data, double proba, bool infered)
{
  if(checkRangeAndDomain(indiv_from, property, data))
  {
    int index = -1;

    index = indiv_from->dataRelationExists(property, data);
    if(index == -1)
    {
      indiv_from->data_relations_.emplace_back(property, data);
      index = indiv_from->data_relations_.size() - 1;
    }

    indiv_from->data_relations_[index].probability = proba;
    indiv_from->data_relations_[index].infered = infered;
    indiv_from->updated_ = true;

    return index;
  }
  else
    throw GraphException("Inconsistency prevented regarding the range or domain of the property");

  return -1;
}

void IndividualGraph::addRelation(IndividualBranch_t* indiv_from, const std::string& property, const std::string& indiv_on)
{
  IndividualBranch_t* branch_from = indiv_from;
  if(branch_from != nullptr)
  {
    IndividualBranch_t* branch_on = findBranchSafe(indiv_on);
    if(branch_on == nullptr)
    {
      ClassBranch_t* test = class_graph_->findBranchSafe(indiv_on);
      if(test != nullptr)
        throw GraphException("object entity does not exists");

      branch_on = findOrCreateBranchSafe(indiv_on);
    }
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);

    ObjectPropertyBranch_t* branch_prop = object_property_graph_->findBranchSafe(property);
    if(branch_prop == nullptr)
    {
      DataPropertyBranch_t* test = data_property_graph_->findBranchSafe(property);
      if(test != nullptr)
        throw GraphException(property + " is a data property");

      std::lock_guard<std::shared_timed_mutex> lock_property(object_property_graph_->mutex_);
      branch_prop = object_property_graph_->newDefaultBranch(property);
    }

    addRelation(branch_from, branch_prop, branch_on);
  }
  else
    throw GraphException("The individual to apply the relation does not exist");
}

void IndividualGraph::addRelation(IndividualBranch_t* indiv_from, const std::string& property, const std::string& type, const std::string& data)
{
  IndividualBranch_t* branch_from = indiv_from;
  if(branch_from != nullptr)
  {
    LiteralNode* literal = data_property_graph_->createLiteral(type + "#" + data);

    DataPropertyBranch_t* branch_prop = data_property_graph_->findBranchSafe(property);
    if(branch_prop == nullptr)
    {
      ObjectPropertyBranch_t* test = object_property_graph_->findBranchSafe(property);
      if(test != nullptr)
        throw GraphException(property + " is an object property");

      std::lock_guard<std::shared_timed_mutex> lock_property(data_property_graph_->mutex_);
      branch_prop = data_property_graph_->newDefaultBranch(property);
    }

    if(checkRangeAndDomain(branch_from, branch_prop, literal))
    {
      conditionalPushBack(branch_from->data_relations_, IndivDataRelationElement_t(branch_prop, literal));
      branch_from->updated_ = true;
    }
    else
      throw GraphException("Inconsistency prevented regarding the range or domain of the property");
  }
  else
    throw GraphException("The individual to apply the relation does not exist");
}

void IndividualGraph::addRelationInvert(const std::string& indiv_from, const std::string& property, IndividualBranch_t* indiv_on)
{
  IndividualBranch_t* branch_on = indiv_on;
  if(branch_on != nullptr)
  {
    IndividualBranch_t* branch_from = findBranchSafe(indiv_from);
    if(branch_from == nullptr)
    {
      ClassBranch_t* test = class_graph_->findBranchSafe(indiv_from);
      if(test != nullptr)
        throw GraphException("The individual to apply the relation does not exist");

      branch_from = findOrCreateBranchSafe(indiv_from);
    }
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);

    ObjectPropertyBranch_t* branch_prop = object_property_graph_->findBranchSafe(property);
    if(branch_prop == nullptr)
    {
      DataPropertyBranch_t* test = data_property_graph_->findBranchSafe(property);
      if(test != nullptr)
        throw GraphException(property + " is a data property");

      std::lock_guard<std::shared_timed_mutex> lock_property(object_property_graph_->mutex_);
      branch_prop = object_property_graph_->newDefaultBranch(property);
    }

    addRelation(branch_from, branch_prop, branch_on);
  }
  else
    throw GraphException("Object entity does not exists");
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removeInheritage(const std::string& indiv, const std::string& class_inherited)
{
  IndividualBranch_t* branch_base = findBranchSafe(indiv);
  ClassBranch_t* branch_inherited = class_graph_->findBranchSafe(class_inherited);
  std::vector<std::pair<std::string, std::string>> explanations;

  if(branch_base == nullptr)
  {
    throw GraphException("The individual entity does not exist");
    return std::vector<std::pair<std::string, std::string>>();
  }
  if(branch_inherited == nullptr)
  {
    throw GraphException("The class_inherited entity does not exist");
    return std::vector<std::pair<std::string, std::string>>();
  }
     
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

  removeInheritage(branch_base, branch_inherited, explanations);

  return explanations;
}

bool IndividualGraph::removeInheritage(IndividualBranch_t* indiv, ClassBranch_t* class_branch, std::vector<std::pair<std::string, std::string>>& explanations, bool protect_stated)
{
  for(size_t i = 0; i < indiv->is_a_.size(); i++)
  {
    if(indiv->is_a_[i].elem == class_branch)
    {
      if((protect_stated == true) && (indiv->is_a_[i].infered == false))
        return false;

      for(auto trace_vect : indiv->is_a_[i].induced_traces)
        trace_vect->eraseGeneric(indiv, nullptr, class_branch);

      auto expl = removeInductions(indiv, indiv->is_a_, i, "isA");
      explanations.insert(explanations.end(), expl.begin(), expl.end());

      indiv->is_a_.erase(i);

      removeFromElemVect(class_branch->individual_childs_, indiv);
      indiv->updated_ = true;
      class_branch->updated_ = true;
      return true;
    }
  }
  return true;
}

void IndividualGraph::addSameAs(const std::string& indiv_1, const std::string& indiv_2)
{
  IndividualBranch_t* branch_1 = findBranchSafe(indiv_1);
  IndividualBranch_t* branch_2 = findBranchSafe(indiv_2);

  if((branch_1 == nullptr) && (branch_2 == nullptr))
    throw GraphException("no known items in the request");

  if(branch_1 == nullptr)
    branch_1 = findOrCreateBranchSafe(indiv_1);
  else if(branch_2 == nullptr)
    branch_2 = findOrCreateBranchSafe(indiv_2);
  else
  {
    std::unordered_set<IndividualBranch_t*> distincts;
    getDistincts(branch_1, distincts);
    if(distincts.find(branch_2) != distincts.end())
      throw GraphException(branch_1->value() + " and " + branch_2->value() + " are distinct");
  }
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  conditionalPushBack(branch_1->same_as_, IndividualElement_t(branch_2));
  conditionalPushBack(branch_2->same_as_, IndividualElement_t(branch_1));

  conditionalPushBack(branch_1->same_as_, IndividualElement_t(branch_1));
  conditionalPushBack(branch_2->same_as_, IndividualElement_t(branch_2));
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removeSameAs(const std::string& indiv_1, const std::string& indiv_2, bool protect_stated)
{
  IndividualBranch_t* branch_1 = findBranchSafe(indiv_1);
  IndividualBranch_t* branch_2 = findBranchSafe(indiv_2);

  if((branch_1 == nullptr) || (branch_2 == nullptr))
  {
    throw GraphException("One of the two individuals used in sameAs relation does not exist");
    return std::vector<std::pair<std::string, std::string>>();
  }

  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  std::vector<std::pair<std::string, std::string>> explanations;

  if(branch_1->same_as_.size() > 0)
  {
    for(size_t i = 0; i < branch_1->same_as_.size(); i++)
    {
      if(branch_1->same_as_[i].elem == branch_2)
      {
        if((protect_stated == true) && (branch_1->same_as_[i].infered == false))
          break;

        auto expl = removeInductions(branch_1, branch_1->same_as_, i, "sameAs");
        explanations.insert(explanations.end(), expl.begin(), expl.end());

        for(size_t j = 0; j < branch_2->same_as_.size(); j++)
        {
          if(branch_2->same_as_[j].elem == branch_1)
          {
            if((protect_stated == true) && (branch_2->same_as_[j].infered == false))
              break;

            expl = removeInductions(branch_2, branch_2->same_as_, j, "sameAs");
            explanations.insert(explanations.end(), expl.begin(), expl.end());

            branch_2->same_as_.erase(j);
            break;
          }
        }
        branch_1->same_as_.erase(i);
        break;
      }
    }
  }

  if(branch_1->same_as_.size() == 1)
    branch_1->same_as_.clear();
    
  if(branch_2->same_as_.size() == 1)
    branch_2->same_as_.clear();

  branch_1->updated_ = true;
  branch_2->updated_ = true;

  return explanations;
}

std::pair<std::vector<std::pair<std::string, std::string>>, bool> IndividualGraph::removeRelation(IndividualBranch_t* branch_from, ObjectPropertyBranch_t* property, IndividualBranch_t* branch_on, bool protect_stated)
{
  std::vector<std::pair<std::string, std::string>> explanations;
  std::unordered_set<ObjectPropertyBranch_t*> down_properties;
  object_property_graph_->getDownPtr(property, down_properties);

  for(size_t i = 0; i < branch_from->object_relations_.size();)
  {
    auto& object_relation = branch_from->object_relations_[i];
    bool applied = false;
    for(auto& down : down_properties)
    {
      if(object_relation.first == down)
      {
        if((branch_on == nullptr) || (object_relation.second == branch_on)) // if branch_on == nullptr we have to remove relations regardless the object
        {
          if((protect_stated == true) && (object_relation.infered == false))
          {
            if(branch_on == nullptr)
              break; // if we have to remove everything we do not return now
            else
              return {{}, false};
          }

          for(auto& trace_vect : object_relation.induced_traces)
            trace_vect->eraseGeneric(branch_from, property, branch_on);

          auto exp_inv = removeRelationInverse(branch_from, object_relation.first, object_relation.second);
          auto exp_sym = removeRelationSymetric(branch_from, object_relation.first, object_relation.second);
          auto exp_ch  = removeInductions(branch_from, branch_from->object_relations_, i);
          explanations.insert(explanations.end(), exp_inv.begin(), exp_inv.end());
          explanations.insert(explanations.end(), exp_sym.begin(), exp_sym.end());
          explanations.insert(explanations.end(), exp_ch.begin(), exp_ch.end());

          object_relation.second->updated_ = true;
          branch_from->object_relations_.erase(i);
          branch_from->updated_ = true;
          applied = true;

          if(branch_on == nullptr)
            break; // if we have to remove everything we do not return now
          else
            return {explanations, true};
        }
      }
    }

    if(applied == false)
      i++;
  }

  return {explanations, true}; // we should reach this place only if we remove relations regardless the object
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removeRelation(const std::string& indiv_from, const std::string& property, const std::string& indiv_on)
{
  IndividualBranch_t* branch_from = findBranchSafe(indiv_from);
  if(branch_from != nullptr)
  {
    ObjectPropertyBranch_t* branch_property = object_property_graph_->findBranchSafe(property);
    if(branch_property != nullptr)
    {
      if(indiv_on != "_")
      {
        IndividualBranch_t* branch_on = findBranchSafe(indiv_on);
        if(branch_on != nullptr)
          return removeRelation(branch_from, branch_property, branch_on).first;
        else
          throw GraphException("The object entity does not exist");
      }
      else
        return removeRelation(branch_from, branch_property, nullptr).first;
    }
    else
      throw GraphException("The property does not exist");
  }
  else
    throw GraphException("The subject entity does not exist");

  return std::vector<std::pair<std::string, std::string>>();
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removeRelation(const std::string& indiv_from, const std::string& property, const std::string& type, const std::string& data)
{
  std::vector<std::pair<std::string, std::string>> explanations;
  IndividualBranch_t* branch_from = findBranchSafe(indiv_from);
  if(branch_from != nullptr)
  {
    bool fuzzy = (type == "_") || (data == "_");
    for(size_t i = 0; i < branch_from->data_relations_.size();)
    {
      if(branch_from->data_relations_[i].first->value() == property)
      {
        if(((type == "_") || (branch_from->data_relations_[i].second->type_ == type)) &&
          ((data == "_") || (branch_from->data_relations_[i].second->value_ == data)))
        {
          auto tmp_expl = removeInductions(branch_from, branch_from->data_relations_, i);
          explanations.insert(explanations.end(), tmp_expl.begin(), tmp_expl.end());
          
          branch_from->data_relations_.erase(i);
          branch_from->updated_ = true;

          if(fuzzy == false)
            return explanations;
        }
        else
          i++;
      }
      else
        i++;
    }
    return explanations;
  }
  else
    throw GraphException("The subject entity does not exist");
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removeRelationInverse(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  std::vector<std::pair<std::string, std::string>> explanations;
  for(auto& invert : property->inverses_)
  {
    for(size_t i = 0; i < indiv_on->object_relations_.size();)
      if((indiv_on->object_relations_[i].first->get() == invert.elem->get()) &&
        (indiv_on->object_relations_[i].second->get() == indiv_from->get()))
        {
          explanations.emplace_back("[DEL]" + indiv_on->value() + "|" + indiv_on->object_relations_[i].first->value() + "|" + indiv_on->object_relations_[i].second->value(),
                                     "[DEL]" + indiv_from->value() + "|" + property->value() + "|" + indiv_on->value());

          auto exp_ch  = removeInductions(indiv_on, indiv_on->object_relations_, i);
          explanations.insert(explanations.end(), exp_ch.begin(), exp_ch.end());

          indiv_on->object_relations_[i].second->updated_ = true;
          indiv_on->object_relations_.erase(i);
          indiv_on->updated_ = true;
        }
      else
        i++;
  }
  return explanations;
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removeRelationSymetric(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  std::vector<std::pair<std::string, std::string>> explanations;
  if(property->properties_.symetric_property_ == true)
  {
    for(size_t i = 0; i < indiv_on->object_relations_.size(); i++)
      if((indiv_on->object_relations_[i].first == property) &&
        (indiv_on->object_relations_[i].second == indiv_from))
        {
          explanations.emplace_back("[DEL]" + indiv_on->value() + "|" + indiv_on->object_relations_[i].first->value() + "|" + indiv_on->object_relations_[i].second->value(),
                                     "[DEL]" + indiv_from->value() + "|" + property->value() + "|" + indiv_on->value());

          auto exp_ch  = removeInductions(indiv_on, indiv_on->object_relations_, i);
          explanations.insert(explanations.end(), exp_ch.begin(), exp_ch.end());
          
          indiv_on->object_relations_[i].second->updated_;
          indiv_on->object_relations_.erase(i);
          indiv_on->updated_ = true;
        }
  }
  return explanations;
}

bool IndividualGraph::checkRangeAndDomain(IndividualBranch_t* from, ObjectPropertyBranch_t* prop, IndividualBranch_t* on)
{
  std::unordered_set<ClassBranch_t*> domain;
  std::unordered_set<ClassBranch_t*> range;
  object_property_graph_->getDomainAndRangePtr(prop, domain, range, 0);

  //DOMAIN
  if(domain.size() != 0)
  {
    std::unordered_set<ClassBranch_t*> up_from;
    getUpPtr(from, up_from);

    auto intersection = class_graph_->checkDomainOrRange(domain, up_from);
    if(intersection.first == false)
    {
      if(intersection.second == nullptr)
        from->flags_["domain"].push_back(prop->value());
      else
        return false;
    }
  }

  //RANGE
  if(range.size() != 0)
  {
    std::unordered_set<ClassBranch_t*> up_on;
    getUpPtr(on, up_on);

    auto intersection = class_graph_->checkDomainOrRange(range, up_on);
    if(intersection.first == false)
    {
      if(intersection.second == nullptr)
        from->flags_["range"].push_back(prop->value());
      else
        return false;
    }
  }

  return true;
}

bool IndividualGraph::checkRangeAndDomain(IndividualBranch_t* from, DataPropertyBranch_t* prop, LiteralNode* data)
{
  //DOMAIN
  std::unordered_set<ClassBranch_t*> domain;
  data_property_graph_->getDomainPtr(prop, domain, 0);

  if(domain.size() != 0)
  {
    std::unordered_set<ClassBranch_t*> up_from;
    getUpPtr(from, up_from);

    auto intersection = class_graph_->checkDomainOrRange(domain, up_from);
    if(intersection.first == false)
    {
      if(intersection.second == nullptr)
        from->flags_["domain"].push_back(prop->value());
      else
        return false;
    }
  }

  //RANGE
  std::unordered_set<std::string> range = data_property_graph_->getRange(prop->value());
  if(range.size() != 0)
  {
    if(range.find(data->type_) == range.end())
      return false;
  }

  return true;
}

void IndividualGraph::deepCopy(const IndividualGraph& other)
{
  for(size_t i = 0; i < other.all_branchs_.size(); i++)
    cpyBranch(other.all_branchs_[i], all_branchs_[i]);
}

void IndividualGraph::cpyBranch(IndividualBranch_t* old_branch, IndividualBranch_t* new_branch)
{
  new_branch->nb_updates_ = old_branch->nb_updates_;
  new_branch->updated_ = old_branch->updated_;
  new_branch->flags_ = old_branch->flags_;

  new_branch->dictionary_ = old_branch->dictionary_;
  new_branch->steady_dictionary_ = old_branch->steady_dictionary_;

  for(const auto& is_a : old_branch->is_a_)
  {
    if(is_a.infered && is_a.induced_traces.size())
      new_branch->updated_ = true;
    else
      new_branch->is_a_.emplace_back(is_a, class_graph_->container_.find(is_a.elem->value()));
  }

  for(const auto& same : old_branch->same_as_)
    new_branch->same_as_.emplace_back(same, container_.find(same.elem->value()));

  for(const auto& distinct : old_branch->distinct_)
    new_branch->distinct_.emplace_back(distinct, container_.find(distinct.elem->value()));

  for(const auto& relation : old_branch->object_relations_)
  {
    // infered relations using traces should not be copied but recomputed
    if(relation.infered && relation.induced_traces.size())
      new_branch->updated_ = true;
    else
    {
      auto prop = object_property_graph_->container_.find(relation.first->value());
      auto on = container_.find(relation.second->value());
      new_branch->object_relations_.emplace_back(relation, prop, on);
    }
  }

  for(const auto& relation : old_branch->data_relations_)
  {
    auto prop = data_property_graph_->container_.find(relation.first->value());
    auto data = relation.second;
    new_branch->data_relations_.emplace_back(relation, prop, data);
  }
}

void IndividualGraph::insertBranchInVectors(IndividualBranch_t* branch)
{
  all_branchs_.push_back(branch);
  if((size_t)branch->get() >= ordered_individuals_.size())
    ordered_individuals_.resize(branch->get() + 1, nullptr);
  ordered_individuals_[branch->get()] = branch;
}

void IndividualGraph::removeBranchInVectors(size_t vector_index)
{
  index_t index = all_branchs_[vector_index]->get();
  all_branchs_.erase(all_branchs_.begin() + vector_index);
  ordered_individuals_[index] = nullptr;
}

} // namespace ontologenius
