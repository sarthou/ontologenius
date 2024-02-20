#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"

#include <algorithm>
#include <iostream>

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

ClassGraph::ClassGraph(IndividualGraph* individual_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph) : OntoGraph(individual_graph)
{
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

ClassGraph::ClassGraph(const ClassGraph& other, IndividualGraph* individual_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph) : OntoGraph(individual_graph)
{
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;

  language_ = other.language_;

  for(auto branch : other.all_branchs_)
  {
    auto class_branch = new ClassBranch_t(branch->value());
    all_branchs_.push_back(class_branch);
  }

  this->container_.load(all_branchs_);
}

ClassBranch_t* ClassGraph::add(const std::string& value, ObjectVectors_t& object_vector)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  //am I created ?
  ClassBranch_t* me = findOrCreateBranch(value);

  /**********************
  ** Class assertion
  **********************/
  //for all my mothers
  for(auto& mother : object_vector.mothers_)
  {
    ClassBranch_t* mother_branch = findOrCreateBranch(mother.elem);

    conditionalPushBack(mother_branch->childs_, ClassElement_t(me, mother.probability, true));
    conditionalPushBack(me->mothers_, ClassElement_t(mother_branch, mother.probability));
  }

  /**********************
  ** Disjoint assertion
  **********************/
  //for all my disjoints
  for(auto& disjoint : object_vector.disjoints_)
  {
    ClassBranch_t* disjoint_branch = findOrCreateBranch(disjoint.elem);

    conditionalPushBack(me->disjoints_, ClassElement_t(disjoint_branch, disjoint.probability));
    conditionalPushBack(disjoint_branch->disjoints_, ClassElement_t(me, disjoint.probability, true));
  }

  /**********************
  ** Object Property assertion
  **********************/
  for(auto& object_relation : object_vector.object_relations_)
    addObjectRelation(me, object_relation);

  /**********************
  ** Data Property assertion
  **********************/
  //for all my properties
  for(auto& data_relation : object_vector.data_relations_)
    addDataRelation(me, data_relation);

  me->setSteadyDictionary(object_vector.dictionary_);
  me->setSteadyMutedDictionary(object_vector.muted_dictionary_);

  mitigate(me);
  return me;
}

void ClassGraph::add(std::vector<std::string>& disjoints)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  for(size_t disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
  {
    //I need to find myself
    ClassBranch_t* me = findOrCreateBranch(disjoints[disjoints_i]);

    //for all my disjoints ...
    for(size_t disjoints_j = 0; disjoints_j < disjoints.size(); disjoints_j++)
    {
      //... excepted me
      if(disjoints_i != disjoints_j)
      {
        ClassBranch_t* disjoint_branch = findOrCreateBranch(disjoints[disjoints_j]);

        conditionalPushBack(me->disjoints_, ClassElement_t(disjoint_branch));
        conditionalPushBack(disjoint_branch->disjoints_, ClassElement_t(me, 1.0, true));
      }
    }
  }
}

/*********
*
* add functions
*
*********/

void ClassGraph::addObjectRelation(ClassBranch_t* me, Pair_t<std::string, std::string>& relation)
{
  ObjectPropertyBranch_t* property_branch = object_property_graph_->findOrCreateBranch(relation.first);
  property_branch->annotation_usage_ = true;

  ClassBranch_t* class_branch = findOrCreateBranch(relation.second);

  me->object_relations_.emplace_back(property_branch, class_branch, relation.probability);
}

void ClassGraph::addDataRelation(ClassBranch_t* me, Pair_t<std::string, std::string>& relation)
{
  DataPropertyBranch_t* property_branch = data_property_graph_->findOrCreateBranch(relation.first);
  property_branch->annotation_usage_ = true;

  auto literal = data_property_graph_->createLiteral(relation.second);
  me->data_relations_.emplace_back(property_branch, literal, relation.probability);
}

/*********
*
* get functions
*
*********/

std::unordered_set<std::string> ClassGraph::getRelationFrom(const std::string& _class, int depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  ClassBranch_t* class_branch = container_.find(_class);
  return getRelationFrom<std::string>(class_branch, depth);
}

std::unordered_set<index_t> ClassGraph::getRelationFrom(index_t _class, int depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  ClassBranch_t* class_branch = container_.find(ValuedNode::table_.get(_class));
  return getRelationFrom<index_t>(class_branch, depth);
}

template<typename T>
std::unordered_set<T> ClassGraph::getRelationFrom(ClassBranch_t* class_branch, int depth)
{
  std::unordered_set<T> res;;
  if(class_branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> up_classes = getUpPtrSafe(class_branch);
    for(ClassBranch_t* it : up_classes)
      getRelationFrom(it, res, depth);
  }

  return res;
}

template<typename T>
void ClassGraph::getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<T>& res, int depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  if(class_branch != nullptr)
  {
    for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
      object_property_graph_->getUp(relation.first, res, depth);

    for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
      data_property_graph_->getUp(relation.first, res, depth);
  }
}

std::unordered_set<std::string> ClassGraph::getRelatedFrom(const std::string& property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  getRelatedFrom(object_properties, data_properties, res);

  return res;
}

std::unordered_set<index_t> ClassGraph::getRelatedFrom(index_t property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<index_t> res;
  getRelatedFrom(object_properties, data_properties, res);

  return res;
}

template<typename T>
void ClassGraph::getRelatedFrom(const std::unordered_set<index_t>& object_properties, const std::unordered_set<index_t>& data_properties, std::unordered_set<T>& res)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  for(auto& branch : all_branchs_)
  {
    for(ClassObjectRelationElement_t& relation : branch->object_relations_)
      for (index_t id : object_properties)
        if(relation.first->get() == id)
          getDown(branch, res);

    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      for (index_t id : data_properties)
        if(relation.first->get() == id)
          getDown(branch, res);
  }
}

std::unordered_set<std::string> ClassGraph::getRelationOn(const std::string& _class, int depth)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  getRelationOnObjectProperties(_class, res, depth);

  if(res.size() == 0)
    getRelationOnDataProperties(_class, res, depth);

  return res;
}

std::unordered_set<index_t> ClassGraph::getRelationOn(index_t _class, int depth)
{
  std::unordered_set<index_t> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  if(_class > 0)
    getRelationOnObjectProperties(ValuedNode::table_.get(_class), res, depth);
  else
    getRelationOnDataProperties(LiteralNode::table_.get(-_class), res, depth);

  return res;
}

template<typename T>
void ClassGraph::getRelationOnObjectProperties(const std::string& _class, std::unordered_set<T>& res, int depth)
{
  ClassBranch_t* class_branch = container_.find(_class);
  if(class_branch != nullptr)
    for(auto& branch : all_branchs_)
      for(ClassObjectRelationElement_t& relation : branch->object_relations_)
        if(relation.second == class_branch)
          object_property_graph_->getUp(relation.first, res, depth);
}

void ClassGraph::getRelationOnDataProperties(const std::string& _class, std::unordered_set<std::string>& res, int depth)
{
  LiteralNode* literal = data_property_graph_->literal_container_.find(_class);

  if(literal != nullptr)
    for(auto& branch : all_branchs_)
      for(ClassDataRelationElement_t& relation : branch->data_relations_)
        if(relation.second == literal)
          data_property_graph_->getUpSafe(relation.first, res, depth);
}

void ClassGraph::getRelationOnDataProperties(const std::string& _class, std::unordered_set<index_t>& res, int depth)
{
  LiteralNode* literal = data_property_graph_->literal_container_.find(_class);

  if(literal != nullptr)
    for(auto& branch : all_branchs_)
      for(ClassDataRelationElement_t& relation : branch->data_relations_)
        if(relation.second == literal)
          data_property_graph_->getUpSafe(relation.first, res, depth);
}

std::unordered_set<std::string> ClassGraph::getRelatedOn(const std::string& property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  for(auto& branch : all_branchs_)
  {
    for(ClassObjectRelationElement_t& relation : branch->object_relations_)
      for (index_t id : object_properties)
        if(relation.first->get() == id)
          res.insert(relation.second->value());

    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      for (index_t id : data_properties)
        if(relation.first->get() == id)
          res.insert(relation.second->value());
  }

  return res;
}

std::unordered_set<index_t> ClassGraph::getRelatedOn(index_t property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<index_t> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  for(auto& branch : all_branchs_)
  {
    for(ClassObjectRelationElement_t& relation : branch->object_relations_)
      for (index_t id : object_properties)
        if(relation.first->get() == id)
          res.insert(relation.second->get());

    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      for (index_t id : data_properties)
        if(relation.first->get() == id)
          res.insert(relation.second->get());
  }

  return res;
}

void ClassGraph::getRelatedOnDataProperties(const std::string& property, std::unordered_set<std::string>& res)
{
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  for(auto& branch : all_branchs_)
  {
    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      for (index_t id : data_properties)
        if(relation.first->get() == id)
          res.insert(relation.second->value());
  }
}

void ClassGraph::getRelatedOnDataProperties(index_t property, std::unordered_set<index_t>& res)
{
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  for(auto& branch : all_branchs_)
  {
    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      for (index_t id : data_properties)
        if(relation.first->get() == id)
          res.insert(relation.second->get());
  }
}

std::unordered_set<std::string> ClassGraph::getRelationWith(const std::string& _class)
{
  std::unordered_set<std::string> res;
  ClassBranch_t* class_branch = container_.find(_class);
  if(class_branch != nullptr)
  {
    std::map<index_t, int> properties;
    std::vector<int> depths;
    std::vector<std::string> tmp_res;
    getRelationWith(class_branch, properties, depths, tmp_res, 0);
    for(auto& it : tmp_res)
      res.insert(it);
  }
  return res;
}

std::unordered_set<index_t> ClassGraph::getRelationWith(index_t _class)
{
  std::unordered_set<index_t> res;
  ClassBranch_t* class_branch = container_.find(ValuedNode::table_.get(_class));
  if(class_branch != nullptr)
  {
    std::map<index_t, int> properties;
    std::vector<int> depths;
    std::vector<index_t> tmp_res;
    getRelationWith(class_branch, properties, depths, tmp_res, 0);
    for(auto& it : tmp_res)
      res.insert(it);
  }
  return res;
}

void ClassGraph::getRelationWith(ClassBranch_t* class_branch, std::map<index_t, int>& properties, std::vector<int>& depths, std::vector<std::string>& res, int depth)
{
  depth++;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  if(class_branch != nullptr)
  {
    for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
    {
      auto it = properties.find(relation.first->get());
      if(it != properties.end())
      {
        int index = properties[relation.first->get()];
        if(depths[index] > depth)
        {
          depths[index] = depth;
          res[index] = relation.second->value();
        }
      }
      else
      {
        properties[relation.first->get()] = res.size();
        depths.push_back(depth);
        res.push_back(relation.second->value());
      }
    }

    for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
    {
      auto it = properties.find(relation.first->get());
      if(it != properties.end())
      {
        int index = properties[relation.first->get()];
        if(depths[index] > depth)
        {
          depths[index] = depth;
          res[index] = relation.second->value();
        }
      }
      else
      {
        properties[relation.first->get()] = res.size();
        depths.push_back(depth);
        res.push_back(relation.second->value());
      }
    }

    std::unordered_set<ClassBranch_t*> up_set = getUpPtrSafe(class_branch, 1);
    for(ClassBranch_t* up : up_set)
      if(up != class_branch)
        getRelationWith(up, properties, depths, res, depth);
  }
}

void ClassGraph::getRelationWith(ClassBranch_t* class_branch, std::map<index_t, int>& properties, std::vector<int>& depths, std::vector<index_t>& res, int depth)
{
  depth++;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  if(class_branch != nullptr)
  {
    for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
    {
      auto it = properties.find(relation.first->get());
      if(it != properties.end())
      {
        int index = properties[relation.first->get()];
        if(depths[index] > depth)
        {
          depths[index] = depth;
          res[index] = relation.second->get();
        }
      }
      else
      {
        properties[relation.first->get()] = res.size();
        depths.push_back(depth);
        res.push_back(relation.second->get());
      }
    }

    for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
    {
      auto it = properties.find(relation.first->get());
      if(it != properties.end())
      {
        int index = properties[relation.first->get()];
        if(depths[index] > depth)
        {
          depths[index] = depth;
          res[index] = relation.second->get();
        }
      }
      else
      {
        properties[relation.first->get()] = res.size();
        depths.push_back(depth);
        res.push_back(relation.second->get());
      }
    }

    std::unordered_set<ClassBranch_t*> up_set = getUpPtrSafe(class_branch, 1);
    for(ClassBranch_t* up : up_set)
      if(up != class_branch)
        getRelationWith(up, properties, depths, res, depth);
  }
}

std::unordered_set<std::string> ClassGraph::getRelatedWith(const std::string& _class)
{
  std::unordered_set<std::string> res;
  std::unordered_set<index_t> do_not_take;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  LiteralNode* literal = data_property_graph_->literal_container_.find(_class);
  ClassBranch_t* class_branch = container_.find(_class);

  for(auto& branch : all_branchs_)
  {
    if(class_branch != nullptr)
      for(ClassObjectRelationElement_t& relation : branch->object_relations_)
        if(relation.second->value() == _class)
          objectGetRelatedWith(branch, relation.first->get(), class_branch->get(), res, do_not_take);

    if(literal != nullptr)
      for(ClassDataRelationElement_t& relation : branch->data_relations_)
        if(relation.second == literal)
          dataGetRelatedWith(branch, relation.first->get(), literal, res, do_not_take);
  }

  for(auto i : do_not_take)
    res.erase(ValuedNode::table_[i]);

  return res;
}

std::unordered_set<index_t> ClassGraph::getRelatedWith(index_t _class)
{
  std::unordered_set<index_t> res;
  std::unordered_set<index_t> do_not_take;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  if(_class > 0)
  {
    ClassBranch_t* class_branch = container_.find(ValuedNode::table_.get(_class));

    if(class_branch != nullptr)
      for(auto& branch : all_branchs_)
      {
        for(ClassObjectRelationElement_t& relation : branch->object_relations_)
          if(relation.second->get() == _class)
            objectGetRelatedWith(branch, relation.first->get(), class_branch->get(), res, do_not_take);
      }
  }
  else
  {
    LiteralNode* literal = data_property_graph_->literal_container_.find(LiteralNode::table_.get(-_class));

    if(literal != nullptr)
      for(auto& branch : all_branchs_)
      {
        for(ClassDataRelationElement_t& relation : branch->data_relations_)
          if(relation.second == literal)
            dataGetRelatedWith(branch, relation.first->get(), literal, res, do_not_take);
      }
  }

  for(auto i : do_not_take)
    res.erase(i);

  return res;
}

template<typename T>
void ClassGraph::dataGetRelatedWith(ClassBranch_t* class_branch, index_t property, LiteralNode* data, std::unordered_set<T>& res, std::unordered_set<index_t>& do_not_take)
{
  if(class_branch != nullptr)
  {
    if(do_not_take.find(class_branch->get()) != do_not_take.end())
      return;

    insert(res, class_branch);

    std::unordered_set<ClassBranch_t*> down_set = getDownPtrSafe(class_branch, 1);
    for(ClassBranch_t* down : down_set)
      if(down != class_branch)
      {
        bool found = false;

        for(ClassDataRelationElement_t& relation : down->data_relations_)
          if(relation.first->get() == property)
            if(relation.second != data)
            {
              found = true;
              getDown(down, do_not_take);
            }

        if(found == false)
          dataGetRelatedWith(down, property, data, res, do_not_take);
      }
  }
}

template<typename T>
void ClassGraph::objectGetRelatedWith(ClassBranch_t* class_branch, index_t property, index_t _class, std::unordered_set<T>& res, std::unordered_set<index_t>& do_not_take)
{
  if(class_branch != nullptr)
  {
    if(do_not_take.find(class_branch->get()) != do_not_take.end())
      return;

    insert(res, class_branch);

    std::unordered_set<ClassBranch_t*> down_set = getDownPtrSafe(class_branch, 1);
    for(ClassBranch_t* down : down_set)
      if(down != class_branch)
      {
        bool found = false;
        for(ClassObjectRelationElement_t& relation : down->object_relations_)
          if(relation.first->get() == property)
            if(relation.second->get() != _class)
            {
              found = true;
              getDown(down, do_not_take);
            }

        if(found == false)
          objectGetRelatedWith(down, property, _class, res, do_not_take);
      }
  }
}

std::unordered_set<std::string> ClassGraph::getFrom(const std::string& param)
{
  std::string _class;
  std::string property;
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    _class = param.substr(0, pose);
    property = param.substr(pose+1);
    return getFrom(_class, property);
  }
  return {};
}

std::unordered_set<std::string> ClassGraph::getFrom(const std::string& _class, const std::string& property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);
  std::unordered_set<index_t> down_classes = getDownId(_class);

  std::unordered_set<std::string> res;
  std::unordered_set<index_t> do_not_take;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  LiteralNode* literal = data_property_graph_->literal_container_.find(_class);

  for(auto& branch : all_branchs_)
  {
    for(ClassObjectRelationElement_t& relation : branch->object_relations_)
      for(index_t class_id : down_classes)
        if(relation.second->get() == class_id)
          for (index_t id : object_properties)
            if(relation.first->get() == id)
              objectGetRelatedWith(branch, relation.first->get(), class_id, res, do_not_take);

    if(literal != nullptr)
      for(ClassDataRelationElement_t& relation :branch->data_relations_)
        if(relation.second == literal)
          for (index_t id : data_properties)
            if(relation.first->get() == id)
              dataGetRelatedWith(branch, relation.first->get(), literal, res, do_not_take);
  }

  for(auto i : do_not_take)
    if(res.find(ValuedNode::table_[i]) != res.end())
      res.erase(ValuedNode::table_[i]);

  return res;
}

std::unordered_set<index_t> ClassGraph::getFrom(index_t _class, index_t property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<index_t> res;
  std::unordered_set<index_t> do_not_take;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  if(_class > 0)
  {
    std::unordered_set<index_t> down_classes = getDownId(_class);

    for(auto& branch : all_branchs_)
    {
      for(ClassObjectRelationElement_t& relation : branch->object_relations_)
        for(index_t class_id : down_classes)
          if(relation.second->get() == class_id)
            for (index_t id : object_properties)
              if(relation.first->get() == id)
                objectGetRelatedWith(branch, relation.first->get(), class_id, res, do_not_take);
    }
  }
  else
  {
    LiteralNode* literal = data_property_graph_->literal_container_.find(LiteralNode::table_.get(-_class));

    for(auto& branch : all_branchs_)
    {
      for(ClassDataRelationElement_t& relation :branch->data_relations_)
        if(relation.second == literal)
          for (index_t id : data_properties)
            if(relation.first->get() == id)
              dataGetRelatedWith(branch, relation.first->get(), literal, res, do_not_take);
    }
  }

  for(auto i : do_not_take)
    res.erase(i);

  return res;
}

std::unordered_set<std::string> ClassGraph::getOn(const std::string& param)
{
  std::string _class;
  std::string property;
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    _class = param.substr(0, pose);
    property = param.substr(pose+1);
    return getOn(_class, property);
  }
  return {};
}

std::unordered_set<std::string> ClassGraph::getOn(const std::string& _class, const std::string& property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  int found_depth = -1;
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  ClassBranch_t* class_branch = container_.find(_class);
  getOn(class_branch, object_properties, data_properties, res, 0, found_depth);

  return res;
}

std::unordered_set<index_t> ClassGraph::getOn(index_t _class, index_t property)
{
  std::unordered_set<index_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<index_t> data_properties = data_property_graph_->getDownId(property);

  int found_depth = -1;
  std::unordered_set<index_t> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  ClassBranch_t* class_branch = container_.find(ValuedNode::table_.get(_class));
  getOn(class_branch, object_properties, data_properties, res, 0, found_depth);

  return res;
}

template<typename T>
void ClassGraph::getOn(ClassBranch_t* class_branch, std::unordered_set<index_t>& object_properties, std::unordered_set<index_t>& data_properties, std::unordered_set<T>& res, uint32_t current_depth, int& found_depth)
{
  if(class_branch != nullptr)
  {
    if(current_depth >= (uint32_t)found_depth)
      return;

    std::unordered_set<T> tmp_res;

    if(object_properties.size())
    {
      for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
        for (index_t id : object_properties)
          if(relation.first->get() == id)
            insert(tmp_res, relation.second);
    }
    else if(data_properties.size())
    {
      for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
        for (index_t id : data_properties)
          if(relation.first->get() == id)
            insert(tmp_res, relation.second);
    }
    else
      return;

    if(tmp_res.size() != 0)
    {
      if(data_properties.size())
      {
        res = std::move(tmp_res);
        found_depth = current_depth;
        return;
      }
      else
        res.insert(tmp_res.begin(), tmp_res.end());
    }

    current_depth++;
    //std::unordered_set<ClassBranch_t*> up_set = getUpPtrSafe(class_branch, 1);
    for(auto& up : class_branch->mothers_)
      if(up.elem != class_branch)
        getOn(up.elem, object_properties, data_properties, res, current_depth, found_depth);
  }
}

std::unordered_set<std::string> ClassGraph::getWith(const std::string& param, int depth)
{
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    std::string first_class = param.substr(0, pose);
    std::string second_class = param.substr(pose+1);
    return getWith(first_class, second_class, depth);
  }
  return {};
}

std::unordered_set<std::string> ClassGraph::getWith(const std::string& first_class, const std::string& second_class, int depth)
{
  std::unordered_set<std::string> res;

  int found_depth = -1;
  uint32_t current_depth = 0;
  std::unordered_set<index_t> do_not_take;
  std::unordered_set<ClassBranch_t*> up_set;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  index_t second_class_index = 0;
  auto second_class_ptr = container_.find(second_class);
  if(second_class_ptr != nullptr)
    second_class_index = second_class_ptr->get();
  else
  {
    auto literal = data_property_graph_->literal_container_.find(second_class);
    if(literal != nullptr)
      second_class_index = literal->get();
  }

  up_set.insert(container_.find(first_class));
  while(up_set.size() > 0)
  {
    std::unordered_set<ClassBranch_t*> next_step;
    for(auto up : up_set)
      getWith(up, second_class_index, res, do_not_take, current_depth, found_depth, depth, next_step);

    up_set = std::move(next_step);
    current_depth++;
  }

  return res;
}

std::unordered_set<index_t> ClassGraph::getWith(index_t first_class, index_t second_class, int depth)
{
  std::unordered_set<index_t> res;

  int found_depth = -1;
  uint32_t current_depth = 0;
  std::unordered_set<index_t> do_not_take;
  std::unordered_set<ClassBranch_t*> up_set;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  up_set.insert(container_.find(ValuedNode::table_.get(first_class)));
  while(up_set.size() > 0)
  {
    std::unordered_set<ClassBranch_t*> next_step;
    for(auto up : up_set)
      getWith(up, second_class, res, do_not_take, current_depth, found_depth, depth, next_step);

    up_set = std::move(next_step);
    current_depth++;
  }

  return res;
}


void ClassGraph::getWith(ClassBranch_t* first_class, index_t second_class, std::unordered_set<std::string>& res, std::unordered_set<index_t>& do_not_take, uint32_t current_depth, int& found_depth, int depth_prop, std::unordered_set<ClassBranch_t*>& next_step)
{
  getWith_(first_class, second_class, res, do_not_take, current_depth, found_depth, depth_prop, next_step);
}

void ClassGraph::getWith(ClassBranch_t* first_class, index_t second_class, std::unordered_set<index_t>& res, std::unordered_set<index_t>& do_not_take, uint32_t current_depth, int& found_depth, int depth_prop, std::unordered_set<ClassBranch_t*>& next_step)
{
  getWith_(first_class, second_class, res, do_not_take, current_depth, found_depth, depth_prop, next_step);
}

template<typename T>
void ClassGraph::getWith_(ClassBranch_t* first_class, index_t second_class, std::unordered_set<T>& res, std::unordered_set<index_t>& do_not_take, uint32_t current_depth, int& found_depth, int depth_prop, std::unordered_set<ClassBranch_t*>& next_step)
{
  if(first_class != nullptr)
  {
    std::unordered_set<T> tmp_res;

    if(second_class > 0)
    {
      for(ClassObjectRelationElement_t& relation : first_class->object_relations_)
      {
        if(relation.second->get() == second_class)
          if(do_not_take.find(relation.first->get()) == do_not_take.end())
            object_property_graph_->getUp(relation.first, tmp_res, depth_prop);
        // We indicate that all properties should no longer be used to avoid overloading relations 
        do_not_take.insert(relation.first->get());
      }
    }
    else
    {
      for(ClassDataRelationElement_t& relation : first_class->data_relations_)
      {
        if(relation.second->get() == second_class)
          if(do_not_take.find(relation.first->get()) == do_not_take.end())
            data_property_graph_->getUp(relation.first, tmp_res, depth_prop);
        // We indicate that all properties should no longer be used to avoid overloading relations 
        do_not_take.insert(relation.first->get());
      }
    }

    if(tmp_res.size() != 0)
      if(current_depth < (uint32_t)found_depth)
      {
        res = std::move(tmp_res);
        found_depth = current_depth;
        return;
      }

    getUpPtr(first_class, next_step, 1);
    next_step.erase(first_class);
  }
}

std::unordered_set<std::string> ClassGraph::getDomainOf(const std::string& _class, int depth)
{
  ClassBranch_t* branch = container_.find(_class);
  std::unordered_set<std::string> res;
  getDomainOf(branch, res, depth);
  return res;
}

std::unordered_set<index_t> ClassGraph::getDomainOf(index_t _class, int depth)
{
  ClassBranch_t* branch = container_.find(ValuedNode::table_.get(_class));
  std::unordered_set<index_t> res;
  getDomainOf(branch, res, depth);
  return res;
}

std::unordered_set<std::string> ClassGraph::getRangeOf(const std::string& _class, int depth)
{
  ClassBranch_t* branch = container_.find(_class);
  std::unordered_set<std::string> res;
  getRangeOf(branch, res, depth);
  return res;
}

std::unordered_set<index_t> ClassGraph::getRangeOf(index_t _class, int depth)
{
  ClassBranch_t* branch = container_.find(ValuedNode::table_.get(_class));
  std::unordered_set<index_t> res;
  getRangeOf(branch, res, depth);
  return res;
}

void ClassGraph::getDomainOf(ClassBranch_t* branch, std::unordered_set<std::string>& res, int depth)
{
  if(branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> up_set = getUpPtrSafe(branch, depth);
    for(auto& prop : object_property_graph_->all_branchs_)
    {
      for(auto& dom : prop->domains_)
        if(up_set.find(dom.elem) != up_set.end())
          res.insert(prop->value());
    }
  }
}

void ClassGraph::getDomainOf(ClassBranch_t* branch, std::unordered_set<index_t>& res, int depth)
{
  if(branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> up_set = getUpPtrSafe(branch, depth);
    for(auto& prop : object_property_graph_->all_branchs_)
    {
      for(auto& dom : prop->domains_)
        if(up_set.find(dom.elem) != up_set.end())
          res.insert(prop->get());
    }
  }
}


void ClassGraph::getRangeOf(ClassBranch_t* branch, std::unordered_set<std::string>& res, int depth)
{
  if(branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> up_set = getUpPtrSafe(branch, depth);
    for(auto& prop : object_property_graph_->all_branchs_)
    {
      for(auto& range : prop->ranges_)
        if(up_set.find(range.elem) != up_set.end())
          res.insert(prop->value());
    }
  }
}

void ClassGraph::getRangeOf(ClassBranch_t* branch, std::unordered_set<index_t>& res, int depth)
{
  if(branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> up_set = getUpPtrSafe(branch, depth);
    for(auto& prop : object_property_graph_->all_branchs_)
    {
      for(auto& range : prop->ranges_)
        if(up_set.find(range.elem) != up_set.end())
          res.insert(prop->get());
    }
  }
}

void ClassGraph::getDownIndividual(ClassBranch_t* branch, std::unordered_set<std::string>& res, bool single_same)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  if(single_same)
  {
    res.reserve(res.size() + branch->individual_childs_.size() * 1.5);
    for(auto& indiv : branch->individual_childs_)
      individual_graph_->getLowestSame(indiv.elem, res);
  }
  else
  {
    for(auto& indiv : branch->individual_childs_)
      individual_graph_->getSame(indiv.elem, res);
  }
}

void ClassGraph::getDownIndividual(ClassBranch_t* branch, std::unordered_set<index_t>& res, bool single_same)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  if(single_same)
  {
    res.reserve(res.size() + branch->individual_childs_.size());
    for(auto& indiv : branch->individual_childs_)
      individual_graph_->getLowestSame(indiv.elem, res);
  }
  else
  {
    res.reserve(res.size() + branch->individual_childs_.size() * 1.5);
    for(auto& indiv : branch->individual_childs_)
      individual_graph_->getSame(indiv.elem, res);
  }
}

std::unordered_set<IndividualBranch_t*> ClassGraph::getDownIndividualPtrSafe(ClassBranch_t* branch, size_t depth)
{
  std::unordered_set<IndividualBranch_t*> res;
  getDownIndividualPtr(branch, res, depth);
  return res;
}

void ClassGraph::getDownIndividualPtr(ClassBranch_t* branch, std::unordered_set<IndividualBranch_t*>& res, size_t depth, size_t current_depth)
{
  for(auto& indiv : branch->individual_childs_)
    res.insert(indiv.elem);

  if(current_depth < depth)
  {
    for(auto& child : branch->childs_)
      getDownIndividualPtr(child.elem, res, depth, current_depth + 1);
  }
}

void ClassGraph::deleteClass(ClassBranch_t* _class)
{
  if(_class != nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);

    // erase indiv from parents
    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(_class, up_set, 1);
    for(auto up : up_set)
    {
      for(size_t i = 0; i < up->childs_.size();)
      {
        if(up->childs_[i].elem == _class)
          up->childs_.erase(up->childs_.begin() + i);
        else
          i++;
      }
    }

    std::unordered_set<ClassBranch_t*> down_set;
    getDownPtr(_class, down_set, 1);
    for(auto down : down_set)
    {
      for(size_t i = 0; i < down->mothers_.size();)
      {
        if(down->mothers_[i].elem == _class)
          down->mothers_.erase(i);
        else
          i++;
      }
    }

    IndividualBranch_t* elem = nullptr;
    for(auto& indiv : _class->individual_childs_)
    {
      elem = indiv.elem;
      for(size_t i = 0; i < elem->is_a_.size();)
      {
        if(elem->is_a_[i].elem == _class)
          elem->is_a_.erase(i);
        else
          i++;
      }
    }

    //erase properties applied to _class
    int index = deleteRelationsOnClass(_class, all_branchs_);

    //delete indiv
    if(index > 0)
      all_branchs_.erase(all_branchs_.begin() + index);

    container_.erase(_class);
    delete _class;
  }
}

int ClassGraph::deleteRelationsOnClass(ClassBranch_t* _class, std::vector<ClassBranch_t*> vect)
{
  int class_index = -1;
  for(size_t class_i = 0; class_i < vect.size(); class_i++)
  {
    if(vect[class_i] == _class)
      class_index = class_i;

    for(size_t i = 0; i < vect[class_i]->object_relations_.size();)
      if(vect[class_i]->object_relations_[i].second == _class)
        vect[class_i]->object_relations_.erase(vect[class_i]->object_relations_.begin() + i);
      else
        i++;
  }
  return class_index;
}

bool ClassGraph::addInheritage(const std::string& branch_base, const std::string& branch_inherited)
{
  ClassBranch_t* branch = findBranchSafe(branch_base);
  if(branch != nullptr)
  {
    ClassBranch_t* inherited = findBranchSafe(branch_inherited);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if(inherited == nullptr)
    {
      IndividualBranch_t* tmp = individual_graph_->findBranchSafe(branch_inherited);
      if(tmp != nullptr)
        inherited = individual_graph_->upgradeToBranch(tmp);
      else
      {
        inherited = new ClassBranch_t(branch_inherited);
        container_.insert(inherited);
        all_branchs_.push_back(inherited);
      }
    }
    if(OntoGraph::addInheritage(branch, inherited))
    {
      std::unordered_set<IndividualBranch_t*> down_individuals;
      std::lock_guard<std::shared_timed_mutex> lock_indiv(individual_graph_->mutex_);
      getDownIndividualPtr(branch, down_individuals);
      for(auto indiv : down_individuals)
        indiv->updated_ = true;
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

void ClassGraph::addRelation(ClassBranch_t* class_from, const std::string& property, const std::string& class_on)
{
  ClassBranch_t* branch_from = class_from;
  if(branch_from != nullptr)
  {
    ClassBranch_t* branch_on = findBranchSafe(class_on);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if(branch_on == nullptr)
    {
      IndividualBranch_t* test = individual_graph_->findBranchSafe(class_on);
      if(test != nullptr)
        throw GraphException("object class does not exists");

      branch_on = new ClassBranch_t(class_on);
      container_.insert(branch_on);
      all_branchs_.push_back(branch_on);
    }

    ObjectPropertyBranch_t* branch_prop = object_property_graph_->findBranchSafe(property);
    if(branch_prop == nullptr)
    {
      DataPropertyBranch_t* test = data_property_graph_->findBranchSafe(property);
      if(test != nullptr)
        throw GraphException(property + " is a data property");

      std::lock_guard<std::shared_timed_mutex> lock_property(object_property_graph_->mutex_);
      branch_prop = object_property_graph_->newDefaultBranch(property);
    }

    if(checkRangeAndDomain(branch_from, branch_prop, branch_on))
      conditionalPushBack(branch_from->object_relations_, ClassObjectRelationElement_t(branch_prop, branch_on));
    else
      throw GraphException("Inconsistency prevented regarding the range or domain of the property");
  }
  else
    throw GraphException("The class to apply the relation does not exist");
}

void ClassGraph::addRelation(ClassBranch_t* class_from, const std::string& property, const std::string& type, const std::string& data)
{
  ClassBranch_t* branch_from = class_from;
  if(branch_from != nullptr)
  {
    LiteralNode* literal_branch = data_property_graph_->createLiteral(type + "#" + data);

    DataPropertyBranch_t* branch_prop = data_property_graph_->findBranchSafe(property);
    if(branch_prop == nullptr)
    {
      ObjectPropertyBranch_t* test = object_property_graph_->findBranchSafe(property);
      if(test != nullptr)
        throw GraphException(property + " is an object property");

      std::lock_guard<std::shared_timed_mutex> lock_property(data_property_graph_->mutex_);
      branch_prop = data_property_graph_->newDefaultBranch(property);
    }

    if(checkRangeAndDomain(branch_from, branch_prop, literal_branch))
      conditionalPushBack(branch_from->data_relations_, ClassDataRelationElement_t(branch_prop, literal_branch));
    else
      throw GraphException("Inconsistency prevented regarding the range or domain of the property");
  }
  else
    throw GraphException("The class to apply the relation does not exist");
}

void ClassGraph::addRelationInvert(const std::string& class_from, const std::string& property, ClassBranch_t* class_on)
{
  ClassBranch_t* branch_on = class_on;
  if(branch_on != nullptr)
  {
    ClassBranch_t* branch_from = findBranchSafe(class_from);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if(branch_from == nullptr)
    {
      IndividualBranch_t* test = individual_graph_->findBranchSafe(class_from);
      if(test != nullptr)
        throw GraphException("The class to apply the relation does not exist");

      branch_from = new ClassBranch_t(class_from);
      container_.insert(branch_from);
      all_branchs_.push_back(branch_from);
    }

    ObjectPropertyBranch_t* branch_prop = object_property_graph_->findBranchSafe(property);
    if(branch_prop == nullptr)
    {
      DataPropertyBranch_t* test = data_property_graph_->findBranchSafe(property);
      if(test != nullptr)
        throw GraphException(property + " is a data property");

      std::lock_guard<std::shared_timed_mutex> lock_property(object_property_graph_->mutex_);
      branch_prop = object_property_graph_->newDefaultBranch(property);
    }

    if(checkRangeAndDomain(branch_from, branch_prop, branch_on))
      conditionalPushBack(branch_from->object_relations_, ClassObjectRelationElement_t(branch_prop, branch_on));
    else
      throw GraphException("Inconsistency prevented regarding the range or domain of the property");
  }
  else
    throw GraphException("Object class does not exists");
}

void ClassGraph::removeRelation(const std::string& class_from, const std::string& property, const std::string& class_on)
{
  ClassBranch_t* branch_from = findBranchSafe(class_from);
  if(branch_from != nullptr)
  {
    for(size_t i = 0; i < branch_from->object_relations_.size();)
    {
      if(branch_from->object_relations_[i].first->value() == property)
      {
        if((class_on == "_") || (branch_from->object_relations_[i].second->value() == class_on))
        {
          branch_from->object_relations_[i].second->updated_ = true;
          branch_from->object_relations_.erase(branch_from->object_relations_.begin() + i);
          branch_from->updated_ = true;
        }
        else
          i++;
      }
      else
        i++;
    }
  }
  else
    throw GraphException("The subject class does not exist");
}

void ClassGraph::removeRelation(const std::string& class_from, const std::string& property, const std::string& type, const std::string& data)
{
  ClassBranch_t* branch_from = findBranchSafe(class_from);
  if(branch_from != nullptr)
  {
    for(size_t i = 0; i < branch_from->data_relations_.size();)
    {
      if(branch_from->data_relations_[i].first->value() == property)
      {
        if(( (type == "_") || (branch_from->data_relations_[i].second->type_ == type)) &&
          ((data == "_") || (branch_from->data_relations_[i].second->value_ == data)))
        {
          branch_from->data_relations_.erase(branch_from->data_relations_.begin() + i);
          branch_from->updated_ = true;
        }
        else
          i++;
      }
      else
        i++;
    }
  }
  else
    throw GraphException("The subject class does not exist");
}

std::pair<bool, ClassBranch_t*> ClassGraph::checkDomainOrRange(const std::unordered_set<ClassBranch_t*>& domain_or_range, const std::unordered_set<ClassBranch_t*>& classes)
{
  ClassBranch_t* intersection = firstIntersection(classes, domain_or_range);
  if(intersection == nullptr)
  {
    // since we cannot prove that it is part of the domain or range,
    // we check if at least it is dijoint or not.
    // intersection will be non-empty if there is a conflict
    intersection = isDisjoint(domain_or_range, classes);
    return {false, intersection};
  }
  else
    return {true, nullptr}; // the class is part of the domain or range as an intersection exists
                            // TODO if multiple domain or range we have to check all of them to be true
}

bool ClassGraph::checkRangeAndDomain(ClassBranch_t* from, ObjectPropertyBranch_t* prop, ClassBranch_t* on)
{
  std::unordered_set<ClassBranch_t*> domain;
  std::unordered_set<ClassBranch_t*> range;
  object_property_graph_->getDomainAndRangePtr(prop, domain, range, 0);

  //DOMAIN
  if(domain.size() != 0)
  {
    std::unordered_set<ClassBranch_t*> up_from;
    getUpPtr(from, up_from);

    auto intersection = checkDomainOrRange(domain, up_from);
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

    auto intersection = checkDomainOrRange(range, up_on);
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

bool ClassGraph::checkRangeAndDomain(ClassBranch_t* from, DataPropertyBranch_t* prop, LiteralNode* data)
{
  //DOMAIN
  std::unordered_set<ClassBranch_t*> domain;
  data_property_graph_->getDomainPtr(prop, domain, 0);

  if(domain.size() != 0)
  {
    std::unordered_set<ClassBranch_t*> up_from;
    getUpPtr(from, up_from);

    auto intersection = checkDomainOrRange(domain, up_from);
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

void ClassGraph::deepCopy(const ClassGraph& other)
{
  for(size_t i = 0; i < other.all_branchs_.size(); i++)
    cpyBranch(other.all_branchs_[i], all_branchs_[i]);
}

void ClassGraph::cpyBranch(ClassBranch_t* old_branch, ClassBranch_t* new_branch)
{
  new_branch->nb_updates_ = old_branch->nb_updates_;
  new_branch->updated_ = old_branch->updated_;
  new_branch->flags_ = old_branch->flags_;

  new_branch->dictionary_ = old_branch->dictionary_;
  new_branch->steady_dictionary_ = old_branch->steady_dictionary_;

  for(const auto& child : old_branch->childs_)
    new_branch->childs_.emplace_back(child, container_.find(child.elem->value()));

  for(const auto& mother : old_branch->mothers_)
  {
    // infered inheritance using traces should not be copied but recomputed
    if(mother.infered && mother.induced_traces.size())
      new_branch->updated_ = true;
    else
      new_branch->mothers_.emplace_back(mother, container_.find(mother.elem->value()));
  }

  for(const auto& disjoint : old_branch->disjoints_)
    new_branch->disjoints_.emplace_back(disjoint, container_.find(disjoint.elem->value()));

  for(const auto& indiv : old_branch->individual_childs_)
    new_branch->individual_childs_.emplace_back(indiv, individual_graph_->container_.find(indiv.elem->value()));

  for(const auto& relation : old_branch->object_relations_)
  {
    auto prop = object_property_graph_->container_.find(relation.first->value());
    auto on = container_.find(relation.second->value());
    new_branch->object_relations_.emplace_back(relation, prop, on);
  }

  for(const auto& relation : old_branch->data_relations_)
  {
    auto prop = data_property_graph_->container_.find(relation.first->value());
    auto data = relation.second;
    new_branch->data_relations_.emplace_back(relation, prop, data);
  }
}

} // namespace ontologenius
