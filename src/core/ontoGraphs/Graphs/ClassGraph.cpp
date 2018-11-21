#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include <iostream>

#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

ClassGraph::ClassGraph(ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph)
{
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

void ClassGraph::add(const std::string& value, ObjectVectors_t& object_vector)
{
  ClassBranch_t* me = nullptr;
  //am I a created mother ?
  amIA(&me, tmp_mothers_, value);

  //am I a created branch ?
  amIA(&me, branchs_, value);

  //am I a created root ?
  amIA(&me, roots_, value);

  //am I created ?
  if(me == nullptr)
    me = new ClassBranch_t(value);

  me->nb_mothers_ += object_vector.mothers_.size();

  //am I a root ?
  if(me->nb_mothers_ == 0)
    roots_.push_back(me);
  else
  {
    //for all my mothers
    for(size_t mothers_i = 0; mothers_i < object_vector.mothers_.size(); mothers_i++)
    {
      bool i_find_my_mother = false;

      //is a root my mother ?
      isMyMother(me, object_vector.mothers_[mothers_i], roots_, i_find_my_mother);

      //is a branch my mother ?
      isMyMother(me, object_vector.mothers_[mothers_i], branchs_, i_find_my_mother);

      //is a tmp mother is mine ?
      isMyMother(me, object_vector.mothers_[mothers_i], tmp_mothers_, i_find_my_mother);

      //I create my mother
      if(!i_find_my_mother)
      {
        ClassBranch_t* my_mother = new struct ClassBranch_t(object_vector.mothers_[mothers_i]);
        my_mother->setSteady_child(me);
        me->setSteady_mother(my_mother);
        tmp_mothers_.push_back(my_mother);
      }
    }

    //but i am also a branch
    branchs_.push_back(me);
  }

  //for all my disjoints
  for(size_t disjoints_i = 0; disjoints_i < object_vector.disjoints_.size(); disjoints_i++)
  {
    bool i_find_my_disjoint = false;

    //is a root my disjoint ?
    isMyDisjoint(me, object_vector.disjoints_[disjoints_i], roots_, i_find_my_disjoint);

    //is a branch my disjoint ?
    isMyDisjoint(me, object_vector.disjoints_[disjoints_i], branchs_, i_find_my_disjoint);

    //is a tmp mother is my disjoint ?
    isMyDisjoint(me, object_vector.disjoints_[disjoints_i], tmp_mothers_, i_find_my_disjoint);

    //I create my disjoint
    if(!i_find_my_disjoint)
    {
      ClassBranch_t* my_disjoint = new struct ClassBranch_t(object_vector.disjoints_[disjoints_i]);
      me->setSteady_disjoint(my_disjoint);
      my_disjoint->disjoints_.push_back(me);
      tmp_mothers_.push_back(my_disjoint); //I put my disjoint as tmp_mother
    }
  }

  me->setSteady_dictionary(object_vector.dictionary_);
  if(me->dictionary_.find("en") == me->dictionary_.end())
    me->dictionary_["en"].push_back(me->value());
}

void ClassGraph::add(std::vector<std::string>& disjoints)
{
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
      me = new struct ClassBranch_t(disjoints[disjoints_i]);
      tmp_mothers_.push_back(me);
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
          ClassBranch_t* my_disjoint = new struct ClassBranch_t(disjoints[disjoints_j]);
          me->setSteady_disjoint(my_disjoint);
          tmp_mothers_.push_back(my_disjoint); //I put my disjoint as tmp_mother
        }
      }
    }
  }
}

std::unordered_set<std::string> ClassGraph::getDisjoint(const std::string& value)
{
  std::unordered_set<std::string> res;

  ClassBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(size_t disjoint_i = 0; disjoint_i < branch->disjoints_.size(); disjoint_i++)
      getDown(branch->disjoints_[disjoint_i], res);

  return res;
}

std::unordered_set<std::string> ClassGraph::select(std::unordered_set<std::string>& on, const std::string& class_selector)
{
  std::unordered_set<std::string> res;
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
  ClassBranch_t* class_branch = container_.find(_class);
  if(class_branch != nullptr)
  {
    for(size_t i = 0; i < class_branch->object_properties_name_.size(); i++)
      object_property_graph_->getUp(class_branch->object_properties_name_[i], res, depth);

    for(size_t i = 0; i < class_branch->data_properties_name_.size(); i++)
      data_property_graph_->getUp(class_branch->data_properties_name_[i], res, depth);
  }
  return res;
}

std::unordered_set<std::string> ClassGraph::getRelatedFrom(const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  for(size_t i = 0; i < all_branchs_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < all_branchs_[i]->object_properties_name_.size(); prop_i++)
      for (uint32_t id : object_properties)
        if(all_branchs_[i]->object_properties_name_[prop_i]->get() == id)
          res.insert(all_branchs_[i]->value());

    for(size_t prop_i = 0; prop_i < all_branchs_[i]->data_properties_name_.size(); prop_i++)
      for (uint32_t id : data_properties)
        if(all_branchs_[i]->data_properties_name_[prop_i]->get() == id)
          res.insert(all_branchs_[i]->value());
  }
  return res;
}

std::unordered_set<std::string> ClassGraph::getRelationOn(const std::string& _class, int depth)
{
  std::unordered_set<std::string> res;
  ClassBranch_t* class_branch = container_.find(_class);
  uint32_t id = class_branch->get();

  for(size_t i = 0; i < all_branchs_.size(); i++)
    for(size_t prop_i = 0; prop_i < all_branchs_[i]->object_properties_on_.size(); prop_i++)
      if(all_branchs_[i]->object_properties_on_[prop_i]->get() == id)
        object_property_graph_->getUp(all_branchs_[i]->object_properties_name_[prop_i], res, depth);

  if(res.size() == 0)
    for(size_t i = 0; i < all_branchs_.size(); i++)
      for(size_t prop_i = 0; prop_i < all_branchs_[i]->data_properties_data_.size(); prop_i++)
        if(all_branchs_[i]->data_properties_data_[prop_i].value_ == _class)
          data_property_graph_->getUp(all_branchs_[i]->data_properties_name_[prop_i], res, depth);

  return res;
}

std::unordered_set<std::string> ClassGraph::getRelatedOn(const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  for(size_t i = 0; i < all_branchs_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < all_branchs_[i]->object_properties_name_.size(); prop_i++)
      for (uint32_t id : object_properties)
        if(all_branchs_[i]->object_properties_name_[prop_i]->get() == id)
          res.insert(all_branchs_[i]->object_properties_on_[prop_i]->value());

    for(size_t prop_i = 0; prop_i < all_branchs_[i]->data_properties_name_.size(); prop_i++)
      for (uint32_t id : data_properties)
        if(all_branchs_[i]->data_properties_name_[prop_i]->get() == id)
          res.insert(all_branchs_[i]->data_properties_data_[prop_i].toString());
  }

  return res;
}

std::unordered_set<std::string> ClassGraph::getRelationWith(const std::string& _class)
{
  std::unordered_set<std::string> res;
  ClassBranch_t* class_branch = container_.find(_class);
  if(class_branch != nullptr)
  {
    for(size_t i = 0; i < class_branch->object_properties_on_.size(); i++)
      res.insert(class_branch->object_properties_on_[i]->value());

    for(size_t i = 0; i < class_branch->data_properties_data_.size(); i++)
      res.insert(class_branch->data_properties_data_[i].toString());
  }
  return res;
}

std::unordered_set<std::string> ClassGraph::getRelatedWith(const std::string& _class)
{
  std::unordered_set<std::string> res;
  for(size_t i = 0; i < all_branchs_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < all_branchs_[i]->object_properties_on_.size(); prop_i++)
      if(all_branchs_[i]->object_properties_on_[prop_i]->value() == _class)
        res.insert(all_branchs_[i]->value());

    for(size_t prop_i = 0; prop_i < all_branchs_[i]->data_properties_data_.size(); prop_i++)
      if(all_branchs_[i]->data_properties_data_[prop_i].value_ == _class)
        res.insert(all_branchs_[i]->value());
  }

  return res;
}

std::unordered_set<std::string> ClassGraph::getFrom(const std::string& param)
{
  std::unordered_set<std::string> res;
  std::string _class;
  std::string property;
  size_t pose = param.find(":");
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
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  for(size_t i = 0; i < all_branchs_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < all_branchs_[i]->object_properties_on_.size(); prop_i++)
      if(all_branchs_[i]->object_properties_on_[prop_i]->value() == _class)
        for (uint32_t id : object_properties)
          if(all_branchs_[i]->object_properties_name_[prop_i]->get() == id)
            res.insert(all_branchs_[i]->value());

    for(size_t prop_i = 0; prop_i < all_branchs_[i]->data_properties_data_.size(); prop_i++)
      if(all_branchs_[i]->data_properties_data_[prop_i].value_ == _class)
        for (uint32_t id : data_properties)
          if(all_branchs_[i]->data_properties_name_[prop_i]->get() == id)
            res.insert(all_branchs_[i]->value());
  }

  return res;
}

std::unordered_set<std::string> ClassGraph::getOn(const std::string& param)
{
  std::unordered_set<std::string> res;
  std::string _class;
  std::string property;
  size_t pose = param.find(":");
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
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  for(size_t i = 0; i < all_branchs_.size(); i++)
    if(all_branchs_[i]->value() == _class)
    {
      for(size_t prop_i = 0; prop_i < all_branchs_[i]->object_properties_name_.size(); prop_i++)
        for (uint32_t id : object_properties)
          if(all_branchs_[i]->object_properties_name_[prop_i]->get() == id)
            res.insert(all_branchs_[i]->object_properties_on_[prop_i]->value());

      for(size_t prop_i = 0; prop_i < all_branchs_[i]->data_properties_name_.size(); prop_i++)
        for (uint32_t id : data_properties)
          if(all_branchs_[i]->data_properties_name_[prop_i]->get() == id)
            res.insert(all_branchs_[i]->data_properties_data_[prop_i].toString());
    }

  return res;
}

std::unordered_set<std::string> ClassGraph::getWith(const std::string& param, int depth)
{
  std::unordered_set<std::string> res;
  size_t pose = param.find(":");
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
  for(size_t i = 0; i < all_branchs_.size(); i++)
    if(all_branchs_[i]->value() == first_class)
    {
      for(size_t indiv_i = 0; indiv_i < all_branchs_[i]->object_properties_on_.size(); indiv_i++)
        if(all_branchs_[i]->object_properties_on_[indiv_i]->value() == second_class)
          object_property_graph_->getUp(all_branchs_[i]->object_properties_name_[indiv_i], res, depth);

      for(size_t indiv_i = 0; indiv_i < all_branchs_[i]->data_properties_data_.size(); indiv_i++)
        if(all_branchs_[i]->data_properties_data_[indiv_i].value_ == second_class)
          data_property_graph_->getUp(all_branchs_[i]->data_properties_name_[indiv_i], res, depth);
    }
  return res;
}
