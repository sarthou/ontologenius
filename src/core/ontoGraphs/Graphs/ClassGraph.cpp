#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"

#include <algorithm>
#include <iostream>

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

ClassGraph::ClassGraph(IndividualGraph* individual_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph)
{
  individual_graph_ = individual_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

ClassGraph::ClassGraph(const ClassGraph& other, IndividualGraph* individual_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph)
{
  individual_graph_ = individual_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;

  language_ = other.language_;

  for(const auto& root : other.roots_)
  {
    auto class_branch = new ClassBranch_t(root.first);
    roots_[root.first] = class_branch;
    all_branchs_.push_back(class_branch);
  }

  for(const auto& branch : other.branchs_)
  {
    auto class_branch = new ClassBranch_t(branch.first);
    branchs_[branch.first] = class_branch;
    all_branchs_.push_back(class_branch);
  }

  this->container_.load(all_branchs_);
}

void ClassGraph::add(const std::string& value, ObjectVectors_t& object_vector, bool direct_load)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  ClassBranch_t* me = nullptr;
  //am I a created mother ?
  amIA(&me, tmp_mothers_, value);

  //am I a created branch ?
  amIA(&me, branchs_, value);

  //am I a created root ?
  amIA(&me, roots_, value);

  //am I created ?
  if(me == nullptr)
  {
    me = new ClassBranch_t(value);
    if(direct_load)
    {
      all_branchs_.push_back(me);
      this->container_.insert(me);
    }
  }

  me->nb_mothers_ += object_vector.mothers_.size();

  //am I a root ?
  if(me->nb_mothers_ == 0)
    roots_[value] = me;
  else
  {
    /**********************
    ** Class assertion
    **********************/
    //for all my mothers
    for(auto& mother : object_vector.mothers_)
    {
      ClassBranch_t* mother_branch = nullptr;
      getInMap(&mother_branch, mother.elem, roots_);
      getInMap(&mother_branch, mother.elem, branchs_);
      getInMap(&mother_branch, mother.elem, tmp_mothers_);
      if(mother_branch == nullptr)
      {
        mother_branch = new ClassBranch_t(mother.elem);
        tmp_mothers_[mother_branch->value()] = mother_branch;
      }

      conditionalPushBack(mother_branch->childs_, ClassElement_t(me, mother.probability, true));
      conditionalPushBack(me->mothers_, ClassElement_t(mother_branch, mother.probability));
    }
    //but i am also a branch
    branchs_[me->value()] = me;
  }

  /**********************
  ** Disjoint assertion
  **********************/
  //for all my disjoints
  for(auto& disjoint : object_vector.disjoints_)
  {
    ClassBranch_t* disjoint_branch = nullptr;
    getInMap(&disjoint_branch, disjoint.elem, roots_);
    getInMap(&disjoint_branch, disjoint.elem, branchs_);
    getInMap(&disjoint_branch, disjoint.elem, tmp_mothers_);
    if(disjoint_branch == nullptr)
    {
      disjoint_branch = new ClassBranch_t(disjoint.elem);
      tmp_mothers_[disjoint_branch->value()] = disjoint_branch;
    }

    conditionalPushBack(me->disjoints_, ClassElement_t(disjoint_branch, disjoint.probability));
    conditionalPushBack(disjoint_branch->disjoints_, ClassElement_t(me, disjoint.probability, true));
  }

  /**********************
  ** Object Property assertion
  **********************/
  for(auto& object_relation : object_vector.object_relations_)
    addObjectProperty(me, object_relation);

  /**********************
  ** Data Property assertion
  **********************/
  //for all my properties
  for(auto& data_relation : object_vector.data_relations_)
    addDataProperty(me, data_relation);

  me->setSteady_dictionary(object_vector.dictionary_);
  me->setSteady_muted_dictionary(object_vector.muted_dictionary_);

  mitigate(me);
}

void ClassGraph::add(std::vector<std::string>& disjoints)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  for(size_t disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
  {
    //I need to find myself
    ClassBranch_t* me = nullptr;
    //Am I a root ?
    amIA(&me, roots_, disjoints[disjoints_i], false);

    //Am I a branch ?
    amIA(&me, branchs_, disjoints[disjoints_i], false);

    //Am I a tmp_mother ?
    amIA(&me, tmp_mothers_, disjoints[disjoints_i], false);

    // I don't exist ? so I will be a tmp_mother
    if(me == nullptr)
    {
      me = new ClassBranch_t(disjoints[disjoints_i]);
      tmp_mothers_[me->value()] = me;
    }

    //for all my disjoints ...
    for(size_t disjoints_j = 0; disjoints_j < disjoints.size(); disjoints_j++)
    {
      //... excepted me
      if(disjoints_i != disjoints_j)
      {
        bool i_find_my_disjoint = false;

        //is a root my disjoint ?
        isMyDisjoint(me, disjoints[disjoints_j], roots_, i_find_my_disjoint, false);

        //is a branch my disjoint ?
        isMyDisjoint(me, disjoints[disjoints_j], branchs_, i_find_my_disjoint, false);

        //is a tmp mother is my disjoint ?
        isMyDisjoint(me, disjoints[disjoints_j], tmp_mothers_, i_find_my_disjoint, false);

        //I create my disjoint
        if(!i_find_my_disjoint)
        {
          auto my_disjoint = new ClassBranch_t(disjoints[disjoints_j]);
          me->disjoints_.emplace_back(my_disjoint);
          my_disjoint->disjoints_.emplace_back(me); // TODO do not save
          tmp_mothers_[my_disjoint->value()] = my_disjoint; //I put my disjoint as tmp_mother
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

void ClassGraph::addObjectProperty(ClassBranch_t* me, Pair_t<std::string, std::string>& relation)
{
  ObjectPropertyBranch_t* property_branch = nullptr;
  getInMap(&property_branch, relation.first, object_property_graph_->roots_);
  getInMap(&property_branch, relation.first, object_property_graph_->branchs_);
  getInMap(&property_branch, relation.first, object_property_graph_->tmp_mothers_);
  if(property_branch == nullptr)
  {
    ObjectPropertyVectors_t empty_vectors;
    object_property_graph_->add(relation.first, empty_vectors);
    getInMap(&property_branch, relation.first, object_property_graph_->roots_);
  }
  property_branch->annotation_usage_ = true;

  ClassBranch_t* class_branch = nullptr;
  getInMap(&class_branch, relation.second, roots_);
  getInMap(&class_branch, relation.second, branchs_);
  getInMap(&class_branch, relation.second, tmp_mothers_);
  if(class_branch == nullptr)
  {
    class_branch = new ClassBranch_t(relation.second);
    tmp_mothers_[class_branch->value()] = class_branch;
  }

  me->object_relations_.emplace_back(property_branch, class_branch, relation.probability);
}

void ClassGraph::addDataProperty(ClassBranch_t* me, Pair_t<std::string, data_t>& relation)
{
  DataPropertyBranch_t* property_branch = nullptr;
  getInMap(&property_branch, relation.first, data_property_graph_->roots_);
  getInMap(&property_branch, relation.first, data_property_graph_->branchs_);
  getInMap(&property_branch, relation.first, data_property_graph_->tmp_mothers_);
  if(property_branch == nullptr)
  {
    DataPropertyVectors_t empty_vectors;
    data_property_graph_->add(relation.first, empty_vectors);
    getInMap(&property_branch, relation.first, data_property_graph_->roots_);
  }
  property_branch->annotation_usage_ = true;

  me->data_relations_.emplace_back(property_branch, relation.second, relation.probability);
}

/*********
*
* get functions
*
*********/

std::unordered_set<std::string> ClassGraph::getDisjoint(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  ClassBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(auto& disjoint : branch->disjoints_)
      getDown(disjoint.elem, res);

  return res;
}

void ClassGraph::getDisjoint(ClassBranch_t* branch, std::unordered_set<ClassBranch_t*>& res)
{
  for(auto& disjoint : branch->disjoints_)
    getDownPtr(disjoint.elem, res);
}

std::unordered_set<std::string> ClassGraph::select(std::unordered_set<std::string>& on, const std::string& class_selector)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  for(const std::string& it : on)
  {
    std::unordered_set<std::string> tmp = getUp(it);
    if(tmp.find(class_selector) != tmp.end())
      res.insert(it);
  }
  return res;
}

std::unordered_set<std::string> ClassGraph::getRelationFrom(const std::string& _class, int depth)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  ClassBranch_t* class_branch = container_.find(_class);
  if(class_branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> up_classes = getUpPtrSafe(class_branch);
    for(ClassBranch_t* it : up_classes)
      getRelationFrom(it, res, depth);
  }

  return res;
}

void ClassGraph::getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<std::string>& res, int depth)
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
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

  std::unordered_set<std::string> res;
  getRelatedFrom(object_properties, data_properties, res);

  return res;
}

void ClassGraph::getRelatedFrom(std::unordered_set<uint32_t>& object_properties, std::unordered_set<uint32_t>& data_properties, std::unordered_set<std::string>& res)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  for(auto& branch : all_branchs_)
  {
    for(ClassObjectRelationElement_t& relation : branch->object_relations_)
      for (uint32_t id : object_properties)
        if(relation.first->get() == id)
        {
          std::unordered_set<ClassBranch_t*> tmp = getDownPtrSafe(branch);
          for(auto tmp_i : tmp)
            res.insert(tmp_i->value());
        }

    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      for (uint32_t id : data_properties)
        if(relation.first->get() == id)
        {
          std::unordered_set<ClassBranch_t*> tmp = getDownPtrSafe(branch);
          for(auto tmp_i : tmp)
            res.insert(tmp_i->value());
        }
  }
}

std::unordered_set<std::string> ClassGraph::getRelationOn(const std::string& _class, int depth)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  ClassBranch_t* class_branch = container_.find(_class);
  if(class_branch != nullptr)
  {
    uint32_t id = class_branch->get();

    for(auto& branch : all_branchs_)
      for(ClassObjectRelationElement_t& relation : branch->object_relations_)
        if(relation.second->get() == id)
          object_property_graph_->getUp(relation.first, res, depth);
  }

  if(res.size() == 0)
    getRelationOnDataProperties(_class, res, depth);

  return res;
}

void ClassGraph::getRelationOnDataProperties(const std::string& _class, std::unordered_set<std::string>& res, int depth)
{
  data_t data_img(_class);

  for(auto& branch : all_branchs_)
    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      if(relation.second == data_img)
        data_property_graph_->getUp(relation.first, res, depth);
}

std::unordered_set<std::string> ClassGraph::getRelatedOn(const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  for(auto& branch : all_branchs_)
  {
    for(ClassObjectRelationElement_t& relation : branch->object_relations_)
      for (uint32_t id : object_properties)
        if(relation.first->get() == id)
          res.insert(relation.second->value());

    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      for (uint32_t id : data_properties)
        if(relation.first->get() == id)
          res.insert(relation.second.toString());
  }

  return res;
}

void ClassGraph::getRelatedOnDataProperties(const std::string& property, std::unordered_set<std::string>& res)
{
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

  for(auto& branch : all_branchs_)
  {
    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      for (uint32_t id : data_properties)
        if(relation.first->get() == id)
          res.insert(relation.second.toString());
  }
}

std::unordered_set<std::string> ClassGraph::getRelationWith(const std::string& _class)
{
  std::unordered_set<std::string> res;
  ClassBranch_t* class_branch = container_.find(_class);
  if(class_branch != nullptr)
  {
    std::map<std::string, int> properties;
    std::vector<int> depths;
    std::vector<std::string> tmp_res;
    getRelationWith(class_branch, properties, depths, tmp_res, 0);
    for(auto& it : tmp_res)
      res.insert(it);
  }
  return res;
}

void ClassGraph::getRelationWith(ClassBranch_t* class_branch, std::map<std::string, int>& properties, std::vector<int>& depths, std::vector<std::string>& res, int depth)
{
  depth++;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  if(class_branch != nullptr)
  {
    for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
    {
      auto it = properties.find(relation.first->value());
      if(it != properties.end())
      {
        int index = properties[relation.first->value()];
        if(depths[index] > depth)
        {
          depths[index] = depth;
          res[index] = relation.second->value();
        }
      }
      else
      {
        properties[relation.first->value()] = res.size();
        depths.push_back(depth);
        res.push_back(relation.second->value());
      }
    }

    for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
    {
      auto it = properties.find(relation.first->value());
      if(it != properties.end())
      {
        int index = properties[relation.first->value()];
        if(depths[index] > depth)
        {
          depths[index] = depth;
          res[index] = relation.second.toString();
        }
      }
      else
      {
        properties[relation.first->value()] = res.size();
        depths.push_back(depth);
        res.push_back(relation.second.toString());
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
  std::unordered_set<uint32_t> doNotTake;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  data_t data_img(_class);

  for(auto& branch : all_branchs_)
  {
    for(ClassObjectRelationElement_t& relation : branch->object_relations_)
      if(relation.second->value() == _class)
        objectGetRelatedWith(branch, relation.first->value(), _class, res, doNotTake);

    for(ClassDataRelationElement_t& relation : branch->data_relations_)
      if(relation.second == data_img)
        dataGetRelatedWith(branch, relation.first->value(), data_img, res, doNotTake);
  }

  for(auto i : doNotTake)
    if(res.find(ValuedNode::table_[i]) != res.end())
      res.erase(ValuedNode::table_[i]);

  return res;
}

void ClassGraph::dataGetRelatedWith(ClassBranch_t* class_branch, const std::string& property, const data_t& data, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake)
{
  if(doNotTake.find(class_branch->get()) != doNotTake.end())
    return;

  if(class_branch != nullptr)
  {
    res.insert(class_branch->value());

    std::unordered_set<ClassBranch_t*> down_set = getDownPtrSafe(class_branch, 1);
    for(ClassBranch_t* down : down_set)
      if(down != class_branch)
      {
        bool found = false;

        for(ClassDataRelationElement_t& relation : down->data_relations_)
          if(relation.first->value() == property)
            if(relation.second != data)
            {
              found = true;
              getDownIdSafe(down, doNotTake);
            }

        if(found == false)
          dataGetRelatedWith(down, property, data, res, doNotTake);
      }
  }
}

void ClassGraph::objectGetRelatedWith(ClassBranch_t* class_branch, const std::string& property, const std::string& _class, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake)
{
  if(doNotTake.find(class_branch->get()) != doNotTake.end())
    return;

  if(class_branch != nullptr)
  {
    res.insert(class_branch->value());

    std::unordered_set<ClassBranch_t*> down_set = getDownPtrSafe(class_branch, 1);
    for(ClassBranch_t* down : down_set)
      if(down != class_branch)
      {
        bool found = false;
        for(ClassObjectRelationElement_t& relation : down->object_relations_)
          if(relation.first->value() == property)
            if(relation.second->value() != _class)
            {
              found = true;
              getDownIdSafe(down, doNotTake);
            }

        if(found == false)
          objectGetRelatedWith(down, property, _class, res, doNotTake);
      }
  }
}

std::unordered_set<std::string> ClassGraph::getFrom(const std::string& param)
{
  std::unordered_set<std::string> res;
  std::string _class;
  std::string property;
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    _class = param.substr(0, pose);
    property = param.substr(pose+1);
    return getFrom(_class, property);
  }
  return res;
}

std::unordered_set<std::string> ClassGraph::getFrom(const std::string& _class, const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> down_classes = getDownIdSafe(_class);

  std::unordered_set<std::string> res;
  std::unordered_set<uint32_t> doNotTake;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  data_t data_img(_class);

  for(auto& branch : all_branchs_)
  {
    for(ClassObjectRelationElement_t& relation : branch->object_relations_)
      for(uint32_t class_id : down_classes)
        if(relation.second->get() == class_id)
          for (uint32_t id : object_properties)
            if(relation.first->get() == id)
              objectGetRelatedWith(branch, relation.first->value(), ValuedNode::table_[class_id], res, doNotTake);

    for(ClassDataRelationElement_t& relation :branch->data_relations_)
      if(relation.second == data_img)
        for (uint32_t id : data_properties)
          if(relation.first->get() == id)
            dataGetRelatedWith(branch, relation.first->value(), data_img, res, doNotTake);
  }

  for(auto i : doNotTake)
    if(res.find(ValuedNode::table_[i]) != res.end())
      res.erase(ValuedNode::table_[i]);

  return res;
}

std::unordered_set<std::string> ClassGraph::getOn(const std::string& param)
{
  std::unordered_set<std::string> res;
  std::string _class;
  std::string property;
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    _class = param.substr(0, pose);
    property = param.substr(pose+1);
    return getOn(_class, property);
  }
  return res;
}

std::unordered_set<std::string> ClassGraph::getOn(const std::string& _class, const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

  int found_depth = -1;
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  ClassBranch_t* class_branch = container_.find(_class);
  getOn(class_branch, object_properties, data_properties, res, 0, found_depth);

  return res;
}

void ClassGraph::getOn(ClassBranch_t* class_branch, std::unordered_set<uint32_t>& object_properties, std::unordered_set<uint32_t>& data_properties, std::unordered_set<std::string>& res, uint32_t current_depth, int& found_depth)
{
  if(class_branch != nullptr)
  {
    std::unordered_set<std::string> tmp_res;

    for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
      for (uint32_t id : object_properties)
        if(relation.first->get() == id)
          tmp_res.insert(relation.second->value());

    if(tmp_res.size() == 0)
      for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
        for (uint32_t id : data_properties)
          if(relation.first->get() == id)
            tmp_res.insert(relation.second.toString());

    if(tmp_res.size() != 0)
      if(current_depth < (uint32_t)found_depth)
      {
        if(data_properties.size())
        {
          res = tmp_res;
          found_depth = current_depth;
          return;
        }
        else
        res.insert(tmp_res.begin(), res.end());
      }

    current_depth++;
    std::unordered_set<ClassBranch_t*> up_set = getUpPtrSafe(class_branch, 1);
    for(ClassBranch_t* up : up_set)
      if(up != class_branch)
        getOn(up, object_properties, data_properties, res, current_depth, found_depth);
  }
}

std::unordered_set<std::string> ClassGraph::getWith(const std::string& param, int depth)
{
  std::unordered_set<std::string> res;
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    std::string first_class = param.substr(0, pose);
    std::string second_class = param.substr(pose+1);
    return getWith(first_class, second_class, depth);
  }
  return res;
}

std::unordered_set<std::string> ClassGraph::getWith(const std::string& first_class, const std::string& second_class, int depth)
{
  std::unordered_set<std::string> res;

  int found_depth = -1;
  uint32_t current_depth = 0;
  std::unordered_set<uint32_t> doNotTake;
  std::unordered_set<ClassBranch_t*> up_set;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  up_set.insert(container_.find(first_class));
  while(up_set.size() > 0)
  {
    std::unordered_set<ClassBranch_t*> next_step;
    for(auto up : up_set)
      getWith(up, second_class, res, doNotTake, current_depth, found_depth, depth, next_step);

    up_set = std::move(next_step);
    current_depth++;
  }

  return res;
}

std::unordered_set<std::string> ClassGraph::getDomainOf(const std::string& _class, int depth)
{
  ClassBranch_t* branch = container_.find(_class);
  return getDomainOf(branch, depth);
}

std::unordered_set<std::string> ClassGraph::getRangeOf(const std::string& _class, int depth)
{
  ClassBranch_t* branch = container_.find(_class);
  return getRangeOf(branch, depth);
}

std::unordered_set<std::string> ClassGraph::getDomainOf(ClassBranch_t* branch, int depth)
{
  std::unordered_set<std::string> res;

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

  return res;
}

std::unordered_set<std::string> ClassGraph::getRangeOf(ClassBranch_t* branch, int depth)
{
  std::unordered_set<std::string> res;

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

  return res;
}

void ClassGraph::getWith(ClassBranch_t* first_class, const std::string& second_class, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake, uint32_t current_depth, int& found_depth, int depth_prop, std::unordered_set<ClassBranch_t*>& next_step)
{
  if(first_class != nullptr)
  {
    std::unordered_set<std::string> tmp_res;
    std::unordered_set<uint32_t> doNotTake_tmp;

    data_t data_img(second_class);

    for(ClassObjectRelationElement_t& relation : first_class->object_relations_)
    {
      doNotTake_tmp.insert(relation.first->get());
      if(relation.second->value() == second_class)
        if(doNotTake.find(relation.first->get()) == doNotTake.end())
          object_property_graph_->getUp(relation.first, tmp_res, depth_prop);
    }

    for(ClassDataRelationElement_t& relation : first_class->data_relations_)
    {
      doNotTake_tmp.insert(relation.first->get());
      if(relation.second == data_img)
        if(doNotTake.find(relation.first->get()) == doNotTake.end())
          data_property_graph_->getUp(relation.first, tmp_res, depth_prop);
    }

    doNotTake.insert(doNotTake_tmp.begin(), doNotTake_tmp.end());

    if(tmp_res.size() != 0)
      if(current_depth < (uint32_t)found_depth)
      {
        res = std::move(tmp_res);
        found_depth = current_depth;
        return;
      }

    current_depth++;
    getUpPtr(first_class, next_step, 1);
    next_step.erase(first_class);
  }
}

std::unordered_set<std::string> ClassGraph::getDownIndividual(ClassBranch_t* branch)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  for(auto& indiv : branch->individual_childs_)
    res.insert(indiv.elem->value());

  return res;
}

void ClassGraph::getDownIndividual(ClassBranch_t* branch, std::unordered_set<std::string>& res)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  for(auto& indiv : branch->individual_childs_)
    res.insert(indiv.elem->value());
}

std::unordered_set<IndividualBranch_t*> ClassGraph::getDownIndividualPtrSafe(ClassBranch_t* branch)
{
  std::unordered_set<IndividualBranch_t*> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);

  for(auto& indiv : branch->individual_childs_)
    res.insert(indiv.elem);

  return res;
}

void ClassGraph::getDownIndividualPtrSafe(ClassBranch_t* branch, std::unordered_set<IndividualBranch_t*>& res)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ClassBranch_t>::mutex_);
  for(auto& indiv : branch->individual_childs_)
    res.insert(indiv.elem);
}

void ClassGraph::deleteClass(ClassBranch_t* _class)
{
  if(_class != nullptr)
  {
    //std::lock_guard<std::shared_timed_mutex> lock(mutex_);
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
          down->mothers_.erase(down->mothers_.begin() + i);
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
          elem->is_a_.erase(elem->is_a_.begin() + i);
        else
          i++;
      }
    }

    //erase properties applied to _class
    int index = deletePropertiesOnClass(_class, all_branchs_);

    //delete indiv
    if(index > 0)
      all_branchs_.erase(all_branchs_.begin() + index);

    container_.erase(_class);
    delete _class;
  }
}

int ClassGraph::deletePropertiesOnClass(ClassBranch_t* _class, std::vector<ClassBranch_t*> vect)
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

void ClassGraph::addLang(std::string& _class, std::string& lang, std::string& name)
{
  ClassBranch_t* branch = findBranch(_class);
  if(branch != nullptr)
  {
    lang = lang.substr(1);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    branch->setSteady_dictionary(lang, name);
    branch->updated_ = true;
  }
}

void ClassGraph::addInheritage(std::string& class_base, std::string& class_inherited)
{
  ClassBranch_t* branch = findBranch(class_base);
  if(branch != nullptr)
  {
    ClassBranch_t* inherited = findBranch(class_inherited);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if(inherited == nullptr)
    {
      IndividualBranch_t* tmp = individual_graph_->findBranch(class_inherited);
      if(tmp != nullptr)
        inherited = individual_graph_->upgradeToBranch(tmp);
      else
      {
        inherited = new ClassBranch_t(class_inherited);
        container_.insert(inherited);
        all_branchs_.push_back(inherited);
      }
    }
    conditionalPushBack(branch->mothers_, ClassElement_t(inherited));
    conditionalPushBack(inherited->childs_, ClassElement_t(branch));
    branch->updated_ = true;
    inherited->updated_ = true;
    mitigate(branch);
  }
}

void ClassGraph::addProperty(ClassBranch_t* class_from, const std::string& property, const std::string& class_on)
{
  ClassBranch_t* branch_from = class_from;
  if(branch_from != nullptr)
  {
    ClassBranch_t* branch_on = findBranch(class_on);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if(branch_on == nullptr)
    {
      IndividualBranch_t* test = individual_graph_->findBranch(class_on);
      if(test != nullptr)
        throw GraphException("object class does not exists");

      branch_on = new ClassBranch_t(class_on);
      container_.insert(branch_on);
      all_branchs_.push_back(branch_on);
    }

    ObjectPropertyBranch_t* branch_prop = object_property_graph_->findBranch(property);
    if(branch_prop == nullptr)
    {
      DataPropertyBranch_t* test = data_property_graph_->findBranch(property);
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

void ClassGraph::addProperty(ClassBranch_t* class_from, const std::string& property, const std::string& type, const std::string& data)
{
  ClassBranch_t* branch_from = class_from;
  if(branch_from != nullptr)
  {
    data_t data_branch(type, data);

    DataPropertyBranch_t* branch_prop = data_property_graph_->findBranch(property);
    if(branch_prop == nullptr)
    {
      ObjectPropertyBranch_t* test = object_property_graph_->findBranch(property);
      if(test != nullptr)
        throw GraphException(property + " is an object property");

      std::lock_guard<std::shared_timed_mutex> lock_property(data_property_graph_->mutex_);
      branch_prop = data_property_graph_->newDefaultBranch(property);
    }

    if(checkRangeAndDomain(branch_from, branch_prop, data_branch))
      conditionalPushBack(branch_from->data_relations_, ClassDataRelationElement_t(branch_prop, data_branch));
    else
      throw GraphException("Inconsistency prevented regarding the range or domain of the property");
  }
  else
    throw GraphException("The class to apply the relation does not exist");
}

void ClassGraph::addPropertyInvert(const std::string& class_from, const std::string& property, ClassBranch_t* class_on)
{
  ClassBranch_t* branch_on = class_on;
  if(branch_on != nullptr)
  {
    ClassBranch_t* branch_from = findBranch(class_from);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if(branch_from == nullptr)
    {
      IndividualBranch_t* test = individual_graph_->findBranch(class_from);
      if(test != nullptr)
        throw GraphException("The class to apply the relation does not exist");

      branch_from = new ClassBranch_t(class_from);
      container_.insert(branch_from);
      all_branchs_.push_back(branch_from);
    }

    ObjectPropertyBranch_t* branch_prop = object_property_graph_->findBranch(property);
    if(branch_prop == nullptr)
    {
      DataPropertyBranch_t* test = data_property_graph_->findBranch(property);
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

void ClassGraph::removeLang(std::string& indiv, std::string& lang, std::string& name)
{
  ClassBranch_t* branch = findBranch(indiv);

  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  lang = lang.substr(1);
  removeFromDictionary(branch->dictionary_.spoken_, lang, name);
  removeFromDictionary(branch->dictionary_.muted_, lang, name);
  removeFromDictionary(branch->steady_dictionary_.spoken_, lang, name);
  removeFromDictionary(branch->steady_dictionary_.muted_, lang, name);
}

void ClassGraph::removeInheritage(std::string& class_base, std::string& class_inherited)
{
  ClassBranch_t* branch_base = findBranch(class_base);
  ClassBranch_t* branch_inherited = findBranch(class_inherited);

  if(branch_base == nullptr)
    return;
  if(branch_inherited == nullptr)
    return;

  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  removeFromElemVect(branch_base->mothers_, branch_inherited);
  removeFromElemVect(branch_inherited->childs_, branch_base);

  branch_base->updated_ = true;
  branch_inherited->updated_ = true;
}

void ClassGraph::removeProperty(std::string& class_from, std::string& property, std::string& class_on)
{
  ClassBranch_t* branch_from = findBranch(class_from);
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

void ClassGraph::removeProperty(std::string& class_from, std::string& property, std::string& type, std::string& data)
{
  ClassBranch_t* branch_from = findBranch(class_from);
  if(branch_from != nullptr)
  {
    for(size_t i = 0; i < branch_from->data_relations_.size();)
    {
      if(branch_from->data_relations_[i].first->value() == property)
      {
        if(( (type == "_") || (branch_from->data_relations_[i].second.type_ == type)) &&
          ((data == "_") || (branch_from->data_relations_[i].second.value_ == data)))
        {
          branch_from->data_relations_.erase(branch_from->data_relations_.begin() + i);
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

bool ClassGraph::checkRangeAndDomain(ClassBranch_t* from, ObjectPropertyBranch_t* prop, ClassBranch_t* on)
{
  std::unordered_set<ClassBranch_t*> up_from;
  getUpPtr(from, up_from);

  std::unordered_set<ObjectPropertyBranch_t*> prop_up;
  object_property_graph_->getUpPtr(prop, prop_up);

  //DOMAIN
  std::unordered_set<ClassBranch_t*> domain;
  for(auto prop : prop_up)
    object_property_graph_->getDomainPtr(prop, domain);

  if(domain.size() != 0)
  {
    ClassBranch_t* intersection = findIntersection(up_from, domain);
    if(intersection == nullptr)
    {
      std::unordered_set<ClassBranch_t*> disjoints;
      for(auto dom : domain)
        getDisjoint(dom, disjoints);
      intersection = findIntersection(up_from, disjoints);

      if(intersection == nullptr)
        from->flags_["domain"].push_back(prop->value());
      else
        return false;
    }
  }

  //RANGE
  std::unordered_set<ClassBranch_t*> up_on;
  getUpPtr(on, up_on);

  std::unordered_set<ClassBranch_t*> range;
  for(auto prop : prop_up)
    object_property_graph_->getRangePtr(prop, range);

  if(range.size() != 0)
  {
    ClassBranch_t* intersection = findIntersection(up_on, range);
    if(intersection == nullptr)
    {
      std::unordered_set<ClassBranch_t*> disjoints;
      for(auto ran : range)
        getDisjoint(ran, disjoints);
      intersection = findIntersection(up_on, disjoints);

      if(intersection == nullptr)
        from->flags_["range"].push_back(prop->value());
      else
        return false;
    }
  }

  return true;
}

bool ClassGraph::checkRangeAndDomain(ClassBranch_t* from, DataPropertyBranch_t* prop, data_t& data)
{
  std::unordered_set<ClassBranch_t*> up_from;
  getUpPtr(from, up_from);

  std::unordered_set<DataPropertyBranch_t*> prop_up;
  data_property_graph_->getUpPtr(prop, prop_up);

  //DOMAIN
  std::unordered_set<ClassBranch_t*> domain;
  for(auto prop : prop_up)
    data_property_graph_->getDomainPtr(prop, domain);

  if(domain.size() != 0)
  {
    ClassBranch_t* intersection = findIntersection(up_from, domain);
    if(intersection == nullptr)
    {
      std::unordered_set<ClassBranch_t*> disjoints;
      for(auto dom : domain)
        getDisjoint(dom, disjoints);
      intersection = findIntersection(up_from, disjoints);

      if(intersection == nullptr)
        from->flags_["range"].push_back(prop->value());
      else
        return false;
    }
  }

  //RANGE
  std::unordered_set<std::string> range = data_property_graph_->getRange(prop->value());
  if(range.size() != 0)
  {
    if(range.find(data.type_) == range.end())
      return false;
  }

  return true;
}

void ClassGraph::deepCopy(const ClassGraph& other)
{
  for(const auto& root : other.roots_)
    cpyBranch(root.second, roots_[root.first]);

  for(const auto& branch : other.branchs_)
    cpyBranch(branch.second, branchs_[branch.first]);
}

void ClassGraph::cpyBranch(ClassBranch_t* old_branch, ClassBranch_t* new_branch)
{
  new_branch->family = old_branch->family;
  new_branch->nb_mothers_ = old_branch->nb_mothers_;

  new_branch->nb_updates_ = old_branch->nb_updates_;
  new_branch->updated_ = old_branch->updated_;
  new_branch->flags_ = old_branch->flags_;

  new_branch->dictionary_ = old_branch->dictionary_;
  new_branch->steady_dictionary_ = old_branch->steady_dictionary_;

  for(const auto& child : old_branch->childs_)
    new_branch->childs_.emplace_back(child, container_.find(child.elem->value()));

  for(const auto& mother : old_branch->mothers_)
    new_branch->mothers_.emplace_back(mother, container_.find(mother.elem->value()));

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
