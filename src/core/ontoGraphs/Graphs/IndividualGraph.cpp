#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include <random>
#include <algorithm>

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

#include "ontoloGenius/core/Algorithms/LevenshteinDistance.h"

namespace ontologenius {

IndividualGraph::IndividualGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

IndividualGraph::~IndividualGraph()
{
  for(size_t i = 0; i < individuals_.size(); i++)
    delete individuals_[i];

  individuals_.clear();
}

void IndividualGraph::close()
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  container_.load(individuals_);
}

void IndividualGraph::add(std::string value, IndividualVectors_t& individual_vector)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  //am I created ?
  IndividualBranch_t* me = nullptr;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    if(individuals_[i]->value() == value)
    {
      me = individuals_[i];
      individuals_.erase(individuals_.begin() + i);
      break;
    }
  }

  if(me == nullptr)
    me = new IndividualBranch_t(value);

  /**********************
  ** Class assertion
  **********************/
  //for all my classes
  for(size_t is_a_i = 0; is_a_i < individual_vector.is_a_.size(); is_a_i++)
  {
    bool i_find_my_is_a_ = false;

    //is a root my class ?
    auto it = class_graph_->roots_.find(individual_vector.is_a_[is_a_i].elem);
    if(it != class_graph_->roots_.end())
    {
      me->setSteady_is_a(ClassElement_t(it->second));
      it->second->setSteady_individual_child(IndividualElement_t(me));
      i_find_my_is_a_ = true;
    }

    //is a branch my class ?
    it = class_graph_->branchs_.find(individual_vector.is_a_[is_a_i].elem);
    if(it != class_graph_->branchs_.end())
    {
      me->setSteady_is_a(ClassElement_t(it->second));
      it->second->setSteady_individual_child(IndividualElement_t(me));
      i_find_my_is_a_ = true;
    }

    //I create my class
    if(!i_find_my_is_a_)
    {
      ObjectVectors_t empty_vectors;
      class_graph_->add(individual_vector.is_a_[is_a_i].elem, empty_vectors);
      it = class_graph_->roots_.find(individual_vector.is_a_[is_a_i].elem);
      if(it != class_graph_->roots_.end())
      {
        me->setSteady_is_a(ClassElement_t(it->second));
        it->second->setSteady_individual_child(IndividualElement_t(me));
        i_find_my_is_a_ = true;
      }
    }
  }

  /**********************
  ** Object Property assertion
  **********************/
  for(size_t property_i = 0; property_i < individual_vector.object_properties_name_.size(); property_i++)
  {
    bool deduced = individual_vector.object_properties_deduced_[property_i];

    //Object Property assertion name
    addObjectPropertyName(me, individual_vector.object_properties_name_[property_i], deduced);

    //Object Property assertion on indiv
    addObjectPropertyOn(me, individual_vector.object_properties_on_[property_i], deduced);

    me->object_properties_deduced_.push_back(deduced);
    me->object_properties_has_induced_.push_back(Triplet());
  }

  /**********************
  ** Data Property assertion name
  **********************/
  //for all my properties
  for(size_t property_i = 0; property_i < individual_vector.data_properties_name_.size(); property_i++)
  {
    bool deduced = individual_vector.data_properties_deduced_[property_i];

    addDataPropertyName(me, individual_vector.data_properties_name_[property_i], deduced);

    data_t data;
    data.value_ = individual_vector.data_properties_value_[property_i];
    data.type_ = individual_vector.data_properties_type_[property_i];
    addDataPropertyData(me, data, deduced);

    me->data_properties_deduced_.push_back(deduced);
  }

  /**********************
  ** Same In Individual
  **********************/
  //for all my inverses
  for(size_t same_i = 0; same_i < individual_vector.same_as_.size(); same_i++)
  {
    bool i_find_my_same = false;

    //is a root my inverse ?
    for(size_t individual_i = 0; individual_i < individuals_.size(); individual_i++)
      if(individual_vector.same_as_[same_i] == individuals_[individual_i]->value())
      {
        me->setSteady_same_as(individuals_[individual_i]);
        individuals_[individual_i]->same_as_.push_back(me);
        i_find_my_same = true;
      }

    //I create my same
    if(!i_find_my_same)
    {
      IndividualBranch_t* my_same = new struct IndividualBranch_t(individual_vector.same_as_[same_i]);
      me->setSteady_same_as(my_same);
      my_same->same_as_.push_back(me);
      individuals_.push_back(my_same);
    }
  }

  me->setSteady_dictionary(individual_vector.dictionary_);
  if(me->dictionary_.find("en") == me->dictionary_.end())
    me->dictionary_["en"].push_back(me->value());
  me->setSteady_muted_dictionary(individual_vector.muted_dictionary_);

  individuals_.push_back(me);
}

void IndividualGraph::add(std::vector<std::string>& distinct)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  for(size_t distinct_i = 0; distinct_i < distinct.size(); distinct_i++)
  {
    //I need to find myself
    IndividualBranch_t* me = nullptr;
    //Am I created ?
    for(size_t individual_i = 0; individual_i < individuals_.size(); individual_i++)
      if(distinct[distinct_i] == individuals_[individual_i]->value())
        me = individuals_[individual_i];

    // I don't exist ?
    if(me == nullptr)
    {
      me = new struct IndividualBranch_t(distinct[distinct_i]);
      individuals_.push_back(me);
    }

    //for all my distincts ...
    for(size_t distinct_j = 0; distinct_j < distinct.size(); distinct_j++)
    {
      //... excepted me
      if(distinct_i != distinct_j)
      {
        bool i_find_my_distinct = false;

        //is my distinct created ?
        for(size_t individual_i = 0; individual_i < individuals_.size(); individual_i++)
          if(distinct[distinct_j] == individuals_[individual_i]->value())
          {
            me->setSteady_distinct(individuals_[individual_i]);
            i_find_my_distinct = true;
          }

        //I create my distinct
        if(!i_find_my_distinct)
        {
          IndividualBranch_t* my_distinct = new struct IndividualBranch_t(distinct[distinct_j]);
          me->setSteady_distinct(my_distinct);
          individuals_.push_back(my_distinct);
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

void IndividualGraph::addObjectPropertyName(IndividualBranch_t* me, std::string& name, bool deduced)
{
  bool i_find_my_properties = false;

  //is a root my properties ?
  auto it = object_property_graph_->roots_.find(name);
  if(it != object_property_graph_->roots_.end())
  {
    if(deduced == false)
      me->setSteady_object_properties_name(it->second);
    else
      me->object_properties_name_.push_back(it->second);
    i_find_my_properties = true;
  }

  //is a branch my properties ?
  if(!i_find_my_properties)
  {
    it = object_property_graph_->branchs_.find(name);
    if(it != object_property_graph_->branchs_.end())
    {
      if(deduced == false)
        me->setSteady_object_properties_name(it->second);
      else
        me->object_properties_name_.push_back(it->second);
      i_find_my_properties = true;
    }
  }

  //I create my properties
  if(!i_find_my_properties)
  {
    ObjectPropertyVectors_t empty_vectors;
    object_property_graph_->add(name, empty_vectors);
    it = object_property_graph_->roots_.find(name);
    if(it != object_property_graph_->roots_.end())
    {
      if(deduced == false)
        me->setSteady_object_properties_name(it->second);
      else
        me->object_properties_name_.push_back(it->second);
      i_find_my_properties = true;
    }
  }
}

void IndividualGraph::addObjectPropertyOn(IndividualBranch_t* me, std::string& name, bool deduced)
{
  bool i_find_my_properties_on = false;

  //is a individual exist ?
  for(size_t indiv_i = 0; indiv_i < individuals_.size(); indiv_i++)
    if(name == individuals_[indiv_i]->value())
    {
      if(deduced == false)
        me->setSteady_object_properties_on(individuals_[indiv_i]);
      else
        me->object_properties_on_.push_back(individuals_[indiv_i]);
      i_find_my_properties_on = true;
    }

  //I create my individual
  if(!i_find_my_properties_on)
  {
    IndividualBranch_t* tmp = new IndividualBranch_t(name);
    individuals_.push_back(tmp);
    if(deduced == false)
      me->setSteady_object_properties_on(tmp);
    else
      me->object_properties_on_.push_back(tmp);
  }
}

void IndividualGraph::addDataPropertyName(IndividualBranch_t* me, std::string& name, bool deduced)
{
  bool i_find_my_properties = false;

  //is a root my properties ?
  auto it = data_property_graph_->roots_.find(name);
  if(it != data_property_graph_->roots_.end())
  {
    if(deduced == false)
      me->setSteady_data_properties_name(it->second);
    else
      me->data_properties_name_.push_back(it->second);
    i_find_my_properties = true;
  }

  //is a branch my properties ?
  if(!i_find_my_properties)
  {
    it = data_property_graph_->branchs_.find(name);
    if(it != data_property_graph_->branchs_.end())
    {
      if(deduced == false)
        me->setSteady_data_properties_name(it->second);
      else
        me->data_properties_name_.push_back(it->second);
      i_find_my_properties = true;
    }
  }

  //I create my properties
  if(!i_find_my_properties)
  {
    DataPropertyVectors_t empty_vectors;
    data_property_graph_->add(name, empty_vectors);
    it = data_property_graph_->roots_.find(name);
    if(it != data_property_graph_->roots_.end())
    {
      if(deduced == false)
        me->setSteady_data_properties_name(it->second);
      else
        me->data_properties_name_.push_back(it->second);
      i_find_my_properties = true;
    }
  }
}

void IndividualGraph::addDataPropertyData(IndividualBranch_t* me, data_t& data, bool deduced)
{
  if(deduced == false)
    me->setSteady_data_properties_data(data);
  else
    me->data_properties_data_.push_back(data);
}

/*********
*
* get functions
*
*********/

std::unordered_set<std::string> IndividualGraph::getSame(const std::string& individual)
{
  return getSameAndClean(container_.find(individual));
}

std::unordered_set<std::string> IndividualGraph::getDistincts(const std::string& individual)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(individual);
  if(indiv != nullptr)
    for(size_t i = 0; i < indiv->distinct_.size(); i++)
    {
      std::unordered_set<std::string> tmp = getSameAndClean(indiv->distinct_[i]);
      res.insert(tmp.begin(), tmp.end());
    }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelationFrom(const std::string& individual, int depth)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(individual);
  if(indiv != nullptr)
  {
    std::unordered_set<IndividualBranch_t*> sames;
    getSame(indiv, sames);
    cleanMarks(sames);
    for(IndividualBranch_t* it : sames)
    {
      for(size_t i = 0; i < it->object_properties_name_.size(); i++)
        object_property_graph_->getUp(it->object_properties_name_[i], res, depth);

      for(size_t i = 0; i < it->data_properties_name_.size(); i++)
        data_property_graph_->getUp(it->data_properties_name_[i], res, depth);

      std::unordered_set<ClassBranch_t*> up_set;
      getUpPtr(it, up_set);
      for(auto up : up_set)
        getRelationFrom(up, res, depth);
    }
  }
  return res;
}

void IndividualGraph::getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<std::string>& res, int depth)
{
  if(class_branch != nullptr)
  {
    for(size_t i = 0; i < class_branch->object_relations_.size(); i++)
      object_property_graph_->getUp(class_branch->object_relations_[i].first, res, depth);

    for(size_t i = 0; i < class_branch->data_properties_name_.size(); i++)
      data_property_graph_->getUp(class_branch->data_properties_name_[i], res, depth);
  }
}

std::unordered_set<std::string> IndividualGraph::getRelatedFrom(const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

  std::unordered_set<std::string> class_res;
  class_graph_->getRelatedFrom(object_properties, data_properties, class_res);

  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_name_.size(); prop_i++)
      for (uint32_t id : object_properties)
        if(individuals_[i]->object_properties_name_[prop_i]->get() == id)
        {
          std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
          res.insert(tmp.begin(), tmp.end());
          break;
        }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_name_.size(); prop_i++)
      for (uint32_t id : data_properties)
        if(individuals_[i]->data_properties_name_[prop_i]->get() == id)
        {
          std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
          res.insert(tmp.begin(), tmp.end());
          break;
        }

    std::unordered_set<std::string> up_set = getUp(individuals_[i], 1);
    for(auto up : up_set)
      if(class_res.find(up) != class_res.end())
      {
        std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
        res.insert(tmp.begin(), tmp.end());
        break;
      }
  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelationOn(const std::string& individual, int depth)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  std::unordered_set<uint32_t> same = getSameId(individual);
  for(uint32_t id : same)
    for(size_t i = 0; i < individuals_.size(); i++)
      for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_on_.size(); prop_i++)
        if(individuals_[i]->object_properties_on_[prop_i]->get() == id)
          object_property_graph_->getUp(individuals_[i]->object_properties_name_[prop_i], res, depth);

  if(res.size() == 0)
  {
    for(size_t i = 0; i < individuals_.size(); i++)
      for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_data_.size(); prop_i++)
        if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
          data_property_graph_->getUp(individuals_[i]->data_properties_name_[prop_i], res, depth);

    class_graph_->getRelationOnDataProperties(individual, res, depth);
  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelatedOn(const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_name_.size(); prop_i++)
      for (uint32_t id : object_properties)
        if(individuals_[i]->object_properties_name_[prop_i]->get() == id)
        {
          std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]->object_properties_on_[prop_i]);
          res.insert(tmp.begin(), tmp.end());
        }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_name_.size(); prop_i++)
      for (uint32_t id : data_properties)
        if(individuals_[i]->data_properties_name_[prop_i]->get() == id)
          res.insert(individuals_[i]->data_properties_data_[prop_i].toString());
  }

  class_graph_->getRelatedOnDataProperties(property, res);

  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelationWith(const std::string& individual)
{
  std::unordered_set<std::string> res;

  std::map<std::string, int> properties;
  std::vector<int> depths;
  std::vector<std::string> tmp_res;

  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  IndividualBranch_t* indiv = container_.find(individual);
  if(indiv != nullptr)
  {
    std::unordered_set<IndividualBranch_t*> sames;
    getSame(indiv, sames);
    cleanMarks(sames);
    for(IndividualBranch_t* it : sames)
    {
      for(size_t i = 0; i < it->object_properties_on_.size(); i++)
      {
        std::unordered_set<IndividualBranch_t*> sames_tmp;
        getSame(it->object_properties_on_[i], sames_tmp);
        std::unordered_set<std::string> tmp = set2set(sames_tmp);
        res.insert(tmp.begin(), tmp.end());

        properties[it->object_properties_name_[i]->value()] = tmp_res.size();
        depths.push_back(0);
        tmp_res.push_back(it->object_properties_on_[i]->value());
      }
      for(size_t i = 0; i < it->data_properties_data_.size(); i++)
      {
        res.insert(it->data_properties_data_[i].toString());

        properties[it->data_properties_name_[i]->value()] = tmp_res.size();
        depths.push_back(0);
        tmp_res.push_back(it->data_properties_data_[i].toString());
      }
    }

    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(indiv, up_set, 1);
    for(auto up : up_set)
      class_graph_->getRelationWith(up, properties, depths, tmp_res, 0);
    for(auto it : tmp_res)
      res.insert(it);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelatedWith(const std::string& individual)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  for(size_t i = 0; i < individuals_.size(); i++)
  {
    bool found = false;
    std::unordered_set<uint32_t> took;

    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_on_.size(); prop_i++)
      if(individuals_[i]->object_properties_on_[prop_i]->value() == individual)
      {
        found = true;
        took.insert(individuals_[i]->object_properties_name_[prop_i]->get());
      }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_data_.size(); prop_i++)
      if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
      {
        found = true;
        took.insert(individuals_[i]->data_properties_name_[prop_i]->get());
      }

    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(individuals_[i], up_set, 1);
    while(up_set.size() > 0)
    {
      std::unordered_set<ClassBranch_t*> next_step;
      for(auto up : up_set)
        found = found || getRelatedWith(up, individual, next_step, took);

      up_set = next_step;
    }

    if(found == true)
    {
      std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
      res.insert(tmp.begin(), tmp.end());
    }
  }

  return res;
}

bool IndividualGraph::getRelatedWith(ClassBranch_t* class_branch, const std::string& data, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<uint32_t>& took)
{
  bool res = false;
  if(class_branch != nullptr)
  {
    for(size_t prop_i = 0; prop_i < class_branch->object_relations_.size(); prop_i++)
    {
      if(class_branch->object_relations_[prop_i].second->value() == data)
        if(took.find(class_branch->object_relations_[prop_i].first->get()) == took.end())
          res = true;
      took.insert(class_branch->object_relations_[prop_i].first->get());
    }

    for(size_t prop_i = 0; prop_i < class_branch->data_properties_name_.size(); prop_i++)
    {
      if(class_branch->data_properties_data_[prop_i].value_ == data)
        if(took.find(class_branch->data_properties_name_[prop_i]->get()) == took.end())
          res = true;
      took.insert(class_branch->data_properties_name_[prop_i]->get());
    }

    class_graph_->getUpPtr(class_branch, next_step, 1);
    next_step.erase(class_branch);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getFrom(const std::string& param)
{
  std::unordered_set<std::string> res;
  std::string individual;
  std::string property;
  size_t pose = param.find(":");
  if(pose != std::string::npos)
  {
    individual = param.substr(0, pose);
    property = param.substr(pose+1);
    return getFrom(individual, property);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getFrom(const std::string& individual, const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  for(size_t i = 0; i < individuals_.size(); i++)
  {
    bool found = false;
    bool defined = false;

    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_name_.size(); prop_i++)
      for (uint32_t id : object_properties)
        if(individuals_[i]->object_properties_name_[prop_i]->get() == id)
        {
          defined = true;
          if(individuals_[i]->object_properties_on_[prop_i]->value() == individual)
          {
            found = true;
            break;
          }
        }

    if(defined == false)
      for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_name_.size(); prop_i++)
        for (uint32_t id : data_properties)
          if(individuals_[i]->data_properties_name_[prop_i]->get() == id)
          {
            defined = true;
            if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
            {
              found = true;
              break;
            }
          }


    if(found == false)
    {
      std::unordered_set<uint32_t> down_classes = class_graph_->getDownIdSafe(individual);
      std::unordered_set<uint32_t> doNotTake;

      std::unordered_set<ClassBranch_t*> up_set;
      getUpPtr(individuals_[i], up_set, 1);
      while(up_set.size() > 0)
      {
        std::unordered_set<ClassBranch_t*> next_step;
        for(auto up : up_set)
          found = found || getFrom(up, object_properties, data_properties, individual, down_classes, next_step, doNotTake);

        up_set = next_step;
      }
    }

    if(found == true)
    {
      std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
      res.insert(tmp.begin(), tmp.end());
    }
  }

  return res;
}

bool IndividualGraph::getFrom(ClassBranch_t* class_branch, std::unordered_set<uint32_t>& object_properties, std::unordered_set<uint32_t>& data_properties, const std::string& data, std::unordered_set<uint32_t>& down_classes, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<uint32_t>& doNotTake)
{
  if(class_branch != nullptr)
  {
    if(doNotTake.find(class_branch->get()) != doNotTake.end())
      return false;

    bool found = false;
    bool defined = false;

    for(size_t prop_i = 0; prop_i < class_branch->object_relations_.size(); prop_i++)
      for (uint32_t id : object_properties)
        if(class_branch->object_relations_[prop_i].first->get() == id)
        {
          defined = true;
          for(uint32_t class_id : down_classes)
            if(class_branch->object_relations_[prop_i].second->get() == class_id)
            {
              found = true;
              break;
            }
        }

    if(defined == false)
      for(size_t prop_i = 0; prop_i < class_branch->data_properties_name_.size(); prop_i++)
        for (uint32_t id : data_properties)
          if(class_branch->data_properties_name_[prop_i]->get() == id)
          {
            defined = true;
            if(class_branch->data_properties_data_[prop_i].value_ == data)
            {
              found = true;
              break;
            }
          }

    if(defined == true)
    {
      class_graph_->getUpIdSafe(class_branch, doNotTake);
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
  std::unordered_set<std::string> res;
  std::string individual;
  std::string property;
  size_t pose = param.find(":");
  if(pose != std::string::npos)
  {
    individual = param.substr(0, pose);
    property = param.substr(pose+1);
    return getOn(individual, property);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getOn(const std::string& individual, const std::string& property)
{
  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(individual);

  if(indiv != nullptr)
  {
    std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
    std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

    for(size_t prop_i = 0; prop_i < indiv->object_properties_name_.size(); prop_i++)
      for (uint32_t id : object_properties)
        if(indiv->object_properties_name_[prop_i]->get() == id)
        {
          std::unordered_set<std::string> tmp = getSameAndClean(indiv->object_properties_on_[prop_i]);
          res.insert(tmp.begin(), tmp.end());
        }

    for(size_t prop_i = 0; prop_i < indiv->data_properties_name_.size(); prop_i++)
      for (uint32_t id : data_properties)
        if(indiv->data_properties_name_[prop_i]->get() == id)
          res.insert(indiv->data_properties_data_[prop_i].toString());

    if(res.size() == 0)
    {
      int found_depth = -1;
      std::unordered_set<ClassBranch_t*> up_set;
      getUpPtr(indiv, up_set, 1);
      for(ClassBranch_t* up : up_set)
        class_graph_->getOn(up, object_properties, data_properties, res, 1, found_depth);
    }
  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::getWith(const std::string& param, int depth)
{
  std::unordered_set<std::string> res;
  size_t pose = param.find(":");
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

  if(indiv != nullptr)
  {
    for(size_t indiv_i = 0; indiv_i < indiv->object_properties_on_.size(); indiv_i++)
      if(indiv->object_properties_on_[indiv_i]->value() == second_individual)
        object_property_graph_->getUp(indiv->object_properties_name_[indiv_i], res, depth);

    for(size_t indiv_i = 0; indiv_i < indiv->data_properties_data_.size(); indiv_i++)
      if(indiv->data_properties_data_[indiv_i].value_ == second_individual)
        data_property_graph_->getUp(indiv->data_properties_name_[indiv_i], res, depth);

    int found_depth = -1;
    uint32_t current_depth = 0;
    std::unordered_set<uint32_t> doNotTake;
    std::unordered_set<ClassBranch_t*> up_set;
    getUpPtr(indiv, up_set, 1);
    while(up_set.size() > 0)
    {
      std::unordered_set<ClassBranch_t*> next_step;
      for(auto up : up_set)
        class_graph_->getWith(up, second_individual, res, doNotTake, current_depth, found_depth, depth, next_step);

      up_set = next_step;
      current_depth++;
    }
  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::getUp(IndividualBranch_t* indiv, int depth, unsigned int current_depth)
{
  current_depth++;
  std::unordered_set<std::string> res;
  if(indiv != nullptr)
  {
    std::unordered_set<IndividualBranch_t*> sames;
    getSame(indiv, sames);
    cleanMarks(sames);
    for(IndividualBranch_t* it : sames)
      for(size_t i = 0; i < it->is_a_.size(); i++)
        class_graph_->getUp(it->is_a_[i].elem, res, depth, current_depth);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getUp(const std::string& individual, int depth)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* indiv = container_.find(individual);
  return getUp(indiv, depth);
}

void IndividualGraph::getUpPtr(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*>& res, int depth, unsigned int current_depth)
{
  current_depth++;
  if(indiv != nullptr)
  {
    std::unordered_set<IndividualBranch_t*> sames;
    getSame(indiv, sames);
    cleanMarks(sames);
    for(IndividualBranch_t* it : sames)
      for(size_t i = 0; i < it->is_a_.size(); i++)
        class_graph_->getUpPtr(it->is_a_[i].elem, res, depth, current_depth);
  }
}

std::unordered_set<uint32_t> IndividualGraph::getSameId(const std::string& individual)
{
  return getSameIdAndClean(container_.find(individual));
}

void IndividualGraph::getSame(IndividualBranch_t* individual, std::unordered_set<IndividualBranch_t*>& res)
{
  if(individual != nullptr)
  {
    res.insert(individual);
    individual->mark = true;
    for(size_t i = 0; i < individual->same_as_.size(); i++)
      if(individual->same_as_[i]->mark == false)
        getSame(individual->same_as_[i], res);
  }
}

std::unordered_set<std::string> IndividualGraph::getSameAndClean(IndividualBranch_t* individual)
{
  std::unordered_set<IndividualBranch_t*> sames;
  getSame(individual, sames);
  cleanMarks(sames);

  std::unordered_set<std::string> res;
  for(IndividualBranch_t* it : sames)
    res.insert(it->value());

  return res;
}

std::unordered_set<uint32_t> IndividualGraph::getSameIdAndClean(IndividualBranch_t* individual)
{
  std::unordered_set<IndividualBranch_t*> sames;
  getSame(individual, sames);
  cleanMarks(sames);

  std::unordered_set<uint32_t> res;
  for(IndividualBranch_t* it : sames)
    res.insert(it->get());

  return res;
}

std::unordered_set<std::string> IndividualGraph::select(std::unordered_set<std::string>& on, const std::string& class_selector)
{
  std::unordered_set<std::string> res;
  std::unordered_set<std::string> classes;

  for(const std::string& it : on)
  {
    IndividualBranch_t* branch = container_.find(it);
    if(branch!= nullptr)
    {
      std::unordered_set<std::string> tmp = getUp(branch);
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

std::string IndividualGraph::getName(const std::string& value)
{
  std::string res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  IndividualBranch_t* branch = container_.find(value);
  if(branch != nullptr)
  {
    if(branch->dictionary_.find(language_) != branch->dictionary_.end())
      if(branch->dictionary_[language_].size())
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

std::vector<std::string> IndividualGraph::getNames(const std::string& value)
{
  std::vector<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  IndividualBranch_t* branch = container_.find(value);
  if(branch != nullptr)
  {
    if(branch->dictionary_.find(this->language_) != branch->dictionary_.end())
      res = branch->dictionary_[this->language_];
    else
      res.push_back(value);
  }

  return res;
}

std::vector<std::string> IndividualGraph::getEveryNames(const std::string& value)
{
  std::vector<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  IndividualBranch_t* branch = container_.find(value);
  if(branch != nullptr)
  {
    if(branch->dictionary_.find(this->language_) != branch->dictionary_.end())
      res = branch->dictionary_[this->language_];
    else
      res.push_back(value);

    if(branch->muted_dictionary_.find(this->language_) != branch->muted_dictionary_.end())
    {
      std::vector<std::string> muted = branch->muted_dictionary_[this->language_];
      res.insert(res.end(), muted.begin(), muted.end());
    }

  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::find(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    if(individuals_[i]->dictionary_.find(language_) != individuals_[i]->dictionary_.end())
      for(size_t dic_i = 0; dic_i < individuals_[i]->dictionary_[language_].size(); dic_i++)
        if(individuals_[i]->dictionary_[language_][dic_i] == value)
          res.insert(individuals_[i]->value());

    if(individuals_[i]->muted_dictionary_.find(language_) != individuals_[i]->muted_dictionary_.end())
      for(size_t dic_i = 0; dic_i < individuals_[i]->muted_dictionary_[language_].size(); dic_i++)
        if(individuals_[i]->muted_dictionary_[language_][dic_i] == value)
          res.insert(individuals_[i]->value());
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::findSub(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::smatch match;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    if(individuals_[i]->dictionary_.find(language_) != individuals_[i]->dictionary_.end())
      for(size_t dic_i = 0; dic_i < individuals_[i]->dictionary_[language_].size(); dic_i++)
      {
        std::regex regex("\\b(" + individuals_[i]->dictionary_[language_][dic_i] + ")([^ ]*)");
        if(std::regex_search(value, match, regex))
          res.insert(individuals_[i]->value());
      }

    if(individuals_[i]->muted_dictionary_.find(language_) != individuals_[i]->muted_dictionary_.end())
      for(size_t dic_i = 0; dic_i < individuals_[i]->muted_dictionary_[language_].size(); dic_i++)
      {
        std::regex regex("\\b(" + individuals_[i]->muted_dictionary_[language_][dic_i] + ")([^ ]*)");
        if(std::regex_search(value, match, regex))
          res.insert(individuals_[i]->value());
      }
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::findRegex(const std::string& regex)
{
  std::unordered_set<std::string> res;
  std::regex base_regex(regex);
  std::smatch match;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    if(individuals_[i]->dictionary_.find(language_) != individuals_[i]->dictionary_.end())
      for(size_t dic_i = 0; dic_i < individuals_[i]->dictionary_[language_].size(); dic_i++)
        if(std::regex_match(individuals_[i]->dictionary_[language_][dic_i], match, base_regex))
          res.insert(individuals_[i]->value());

    if(individuals_[i]->muted_dictionary_.find(language_) != individuals_[i]->muted_dictionary_.end())
      for(size_t dic_i = 0; dic_i < individuals_[i]->muted_dictionary_[language_].size(); dic_i++)
        if(std::regex_match(individuals_[i]->muted_dictionary_[language_][dic_i], match, base_regex))
          res.insert(individuals_[i]->value());
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::findFuzzy(const std::string& value, double threshold)
{
  double lower_cost = 100000;
  double tmp_cost = 100000;
  std::unordered_set<std::string> res;

  LevenshteinDistance dist;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(auto branch : individuals_)
  {
    if(branch->dictionary_.find(this->language_) != branch->dictionary_.end())
      for(size_t i = 0; i < branch->dictionary_[this->language_].size(); i++)
        if((tmp_cost = dist.get(branch->dictionary_[this->language_][i], value)) <= lower_cost)
        {
          if(tmp_cost != lower_cost)
          {
            lower_cost = tmp_cost;
            res.clear();
          }
          res.insert(branch->dictionary_[this->language_][i]);
        }

    if(branch->muted_dictionary_.find(this->language_) != branch->muted_dictionary_.end())
      for(size_t i = 0; i < branch->muted_dictionary_[this->language_].size(); i++)
        if((tmp_cost = dist.get(branch->muted_dictionary_[this->language_][i], value)) <= lower_cost)
        {
          if(tmp_cost != lower_cost)
          {
            lower_cost = tmp_cost;
            res.clear();
          }
          res.insert(branch->muted_dictionary_[this->language_][i]);
        }
  }

  if(lower_cost > threshold)
    res.clear();

  return res;
}

bool IndividualGraph::touch(const std::string& value)
{
  bool res = false;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  IndividualBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    res = true;

  return res;
}

std::unordered_set<std::string> IndividualGraph::getType(const std::string& class_selector)
{
  std::shared_lock<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

  std::unordered_set<std::string> res;
  ClassBranch_t* class_branch = class_graph_->container_.find(class_selector);
  if(class_branch != nullptr)
  {
    std::unordered_set<ClassBranch_t*> down_set = class_graph_->getDownPtrSafe(class_branch);
    for(auto down : down_set)
      class_graph_->getDownIndividual(down, res);
  }


  return res;
}

void IndividualGraph::cleanMarks(std::unordered_set<IndividualBranch_t*>& indSet)
{
  for(IndividualBranch_t* it : indSet)
    it->mark = false;
}

std::unordered_set<std::string> IndividualGraph::set2set(std::unordered_set<IndividualBranch_t*>& indSet, bool clean)
{
  std::unordered_set<std::string> res;
  for(IndividualBranch_t* it : indSet)
  {
    if(clean)
      it->mark = false;
    res.insert(it->value());
  }
  return res;
}

ClassBranch_t* IndividualGraph::upgradeToBranch(IndividualBranch_t* indiv)
{
  if(indiv != nullptr)
  {
    ClassBranch_t* class_branch = new ClassBranch_t(indiv->value());
    class_branch->mothers_ = std::move(indiv->is_a_);
    class_branch->data_properties_name_ = indiv->data_properties_name_;
    class_branch->data_properties_data_ = indiv->data_properties_data_;
    class_branch->data_properties_deduced_ = indiv->data_properties_deduced_;

    class_branch->steady_.mothers_ = std::move(indiv->steady_.is_a_);
    class_branch->steady_.data_properties_name_ = indiv->steady_.data_properties_name_;
    class_branch->steady_.data_properties_data_ = indiv->steady_.data_properties_data_;

    class_graph_->container_.insert(class_branch);
    class_graph_->all_branchs_.push_back(class_branch);
    redirectDeleteIndividual(indiv, class_branch);

    return class_branch;
  }
  return nullptr;
}

void IndividualGraph::createIndividual(std::string& name)
{
  IndividualBranch_t* indiv = findBranch(name);
  if(indiv == nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    indiv = new IndividualBranch_t(name);
    container_.insert(indiv);
    individuals_.push_back(indiv);
  }
}

void IndividualGraph::deleteIndividual(IndividualBranch_t* indiv)
{
  if(indiv != nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

    // erase indiv from parents
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

    for(auto up : up_set)
    {
      for(size_t i = 0; i < up->steady_.individual_childs_.size();)
      {
        if(up->steady_.individual_childs_[i] == indiv)
          up->steady_.individual_childs_.erase(up->steady_.individual_childs_.begin() + i);
        else
          i++;
      }
    }

    //erase properties applied to indiv
    size_t indiv_index = 0;
    for(size_t indiv_i = 0; indiv_i < individuals_.size(); indiv_i++)
    {
      if(individuals_[indiv_i] == indiv)
        indiv_index = indiv_i;

      for(size_t i = 0; i < individuals_[indiv_i]->object_properties_on_.size();)
        if(individuals_[indiv_i]->object_properties_on_[i] == indiv)
        {
          individuals_[indiv_i]->object_properties_on_.erase(individuals_[indiv_i]->object_properties_on_.begin() + i);
          individuals_[indiv_i]->object_properties_name_.erase(individuals_[indiv_i]->object_properties_name_.begin() + i);
          individuals_[indiv_i]->object_properties_deduced_.erase(individuals_[indiv_i]->object_properties_deduced_.begin() + i);
          individuals_[indiv_i]->object_properties_has_induced_.erase(individuals_[indiv_i]->object_properties_has_induced_.begin() + i);
        }
        else
          i++;

      for(size_t i = 0; i < individuals_[indiv_i]->steady_.object_properties_on_.size();)
        if(individuals_[indiv_i]->steady_.object_properties_on_[i] == indiv)
        {
          individuals_[indiv_i]->steady_.object_properties_on_.erase(individuals_[indiv_i]->steady_.object_properties_on_.begin() + i);
          individuals_[indiv_i]->steady_.object_properties_name_.erase(individuals_[indiv_i]->steady_.object_properties_name_.begin() + i);
          individuals_[indiv_i]->steady_.object_properties_deduced_.erase(individuals_[indiv_i]->steady_.object_properties_deduced_.begin() + i);
        }
        else
          i++;
    }

    //delete indiv
    individuals_.erase(individuals_.begin() + indiv_index);
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
    getUpPtr(indiv, up_set, 1);

    for(auto up : up_set)
    {
      for(size_t i = 0; i < up->individual_childs_.size();)
      {
        if(up->individual_childs_[i] == indiv)
        {
          up->individual_childs_.erase(up->individual_childs_.begin() + i);
          up->childs_.push_back(_class);
        }
        else
          i++;
      }
    }

    for(auto up : up_set)
    {
      for(size_t i = 0; i < up->steady_.individual_childs_.size();)
      {
        if(up->steady_.individual_childs_[i] == indiv)
        {
          up->steady_.individual_childs_.erase(up->steady_.individual_childs_.begin() + i);
          up->steady_.childs_.push_back(_class);
        }
        else
          i++;
      }
    }

    //erase properties applied to indiv
    size_t indiv_index = 0;
    for(size_t indiv_i = 0; indiv_i < individuals_.size(); indiv_i++)
    {
      if(individuals_[indiv_i] == indiv)
        indiv_index = indiv_i;

      for(size_t i = 0; i < individuals_[indiv_i]->object_properties_on_.size();)
        if(individuals_[indiv_i]->object_properties_on_[i] == indiv)
        {
          individuals_[indiv_i]->object_properties_on_.erase(individuals_[indiv_i]->object_properties_on_.begin() + i);
          individuals_[indiv_i]->object_properties_name_.erase(individuals_[indiv_i]->object_properties_name_.begin() + i);
          individuals_[indiv_i]->object_properties_deduced_.erase(individuals_[indiv_i]->object_properties_deduced_.begin() + i);
          individuals_[indiv_i]->object_properties_has_induced_.erase(individuals_[indiv_i]->object_properties_has_induced_.begin() + i);
        }
        else
          i++;

      for(size_t i = 0; i < individuals_[indiv_i]->steady_.object_properties_on_.size();)
        if(individuals_[indiv_i]->steady_.object_properties_on_[i] == indiv)
        {
          individuals_[indiv_i]->steady_.object_properties_on_.erase(individuals_[indiv_i]->steady_.object_properties_on_.begin() + i);
          individuals_[indiv_i]->steady_.object_properties_name_.erase(individuals_[indiv_i]->steady_.object_properties_name_.begin() + i);
          individuals_[indiv_i]->steady_.object_properties_deduced_.erase(individuals_[indiv_i]->steady_.object_properties_deduced_.begin() + i);
        }
        else
          i++;
    }

    //delete indiv
    individuals_.erase(individuals_.begin() + indiv_index);
    container_.erase(indiv);
    delete indiv;
  }
}

void IndividualGraph::addLang(std::string& indiv, std::string& lang, std::string& name)
{
  IndividualBranch_t* branch = findBranch(indiv);
  if(branch != nullptr)
  {
    lang = lang.substr(1);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    branch->setSteady_dictionary(lang, name);
    branch->updated_ = true;
  }
}

void IndividualGraph::addInheritage(std::string& indiv, std::string& class_inherited)
{
  IndividualBranch_t* branch = findBranch(indiv);
  if(branch != nullptr)
  {
    ClassBranch_t* inherited = class_graph_->findBranch(class_inherited);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);
    if(inherited == nullptr)
    {
      IndividualBranch_t* tmp = findBranchUnsafe(class_inherited);
      if(tmp != nullptr)
        inherited = upgradeToBranch(tmp);
      else
      {
        inherited = new ClassBranch_t(class_inherited);
        class_graph_->container_.insert(inherited);
        class_graph_->all_branchs_.push_back(inherited);
      }
    }
    branch->setSteady_is_a(inherited);
    inherited->setSteady_individual_child(branch);
    branch->updated_ = true;
    inherited->updated_ = true;
  }
}

void IndividualGraph::addInheritageInvert(std::string& indiv, std::string& class_inherited)
{
  ClassBranch_t* inherited = class_graph_->findBranch(class_inherited);
  if(inherited != nullptr)
  {
    IndividualBranch_t* branch = findBranch(indiv);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);
    if(branch == nullptr)
    {
      branch = new IndividualBranch_t(indiv);
      container_.insert(branch);
      individuals_.push_back(branch);
    }

    branch->setSteady_is_a(inherited);
    inherited->setSteady_individual_child(branch);
    branch->updated_ = true;
    inherited->updated_ = true;
  }
}

void IndividualGraph::addInheritageInvertUpgrade(std::string& indiv, std::string& class_inherited)
{
  IndividualBranch_t* tmp = findBranch(class_inherited);
  if(tmp != nullptr)
  {
    ClassBranch_t* inherited = upgradeToBranch(tmp);
    IndividualBranch_t* branch = findBranch(indiv);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);
    if(branch == nullptr)
    {
      branch = new IndividualBranch_t(indiv);
      container_.insert(branch);
      individuals_.push_back(branch);
    }

    branch->setSteady_is_a(inherited);
    inherited->setSteady_individual_child(branch);
    branch->updated_ = true;
    inherited->updated_ = true;
  }
}

bool IndividualGraph::addProperty(std::string& indiv_from, std::string& property, std::string& indiv_on)
{
  IndividualBranch_t* branch_from = findBranch(indiv_from);
  if(branch_from != nullptr)
  {
    IndividualBranch_t* branch_on = findBranch(indiv_on);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if(branch_on == nullptr)
    {
      ClassBranch_t* test = class_graph_->findBranch(indiv_on);
      if(test != nullptr)
        return false; // TODO ERR

      branch_on = new IndividualBranch_t(indiv_on);
      container_.insert(branch_on);
      individuals_.push_back(branch_on);
    }

    ObjectPropertyBranch_t* branch_prop = object_property_graph_->findBranch(property);
    if(branch_prop == nullptr)
    {
      DataPropertyBranch_t* test = data_property_graph_->findBranch(property);
      if(test != nullptr)
        return false; // TODO ERR

      std::lock_guard<std::shared_timed_mutex> lock_property(object_property_graph_->mutex_);
      branch_prop = new ObjectPropertyBranch_t(property);
      object_property_graph_->container_.insert(branch_prop);
      object_property_graph_->all_branchs_.push_back(branch_prop);
    }

    if(checkRangeAndDomain(branch_from, branch_prop, branch_on))
    {
      int index = -1;
      index = branch_from->ObjectPropertyExistSteady(branch_prop, branch_on);
      if(index == -1)
      {
        branch_from->steady_.object_properties_name_.push_back(branch_prop);
        branch_from->steady_.object_properties_on_.push_back(branch_on);
      }

      index = branch_from->ObjectPropertyExist(branch_prop, branch_on);
      if(index == -1)
      {
        branch_from->object_properties_name_.push_back(branch_prop);
        branch_from->object_properties_on_.push_back(branch_on);
        branch_from->object_properties_deduced_.push_back(false);
        branch_from->object_properties_has_induced_.push_back(Triplet());
      }
      else
        branch_from->object_properties_deduced_[index] = false;

      branch_from->updated_ = true;
      setObjectPropertiesUpdated(branch_from->object_properties_on_);
      return true;
    }
    else
      return false;
  }
  return false;
}

bool IndividualGraph::addProperty(std::string& indiv_from, std::string& property, std::string& type, std::string& data)
{
  IndividualBranch_t* branch_from = findBranch(indiv_from);
  if(branch_from != nullptr)
  {
    data_t data_branch;
    data_branch.value_ = data;
    data_branch.type_ = type;

    DataPropertyBranch_t* branch_prop = data_property_graph_->findBranch(property);
    if(branch_prop == nullptr)
    {
      ObjectPropertyBranch_t* test = object_property_graph_->findBranch(property);
      if(test != nullptr)
        return false; // TODO ERR

      std::lock_guard<std::shared_timed_mutex> lock_property(data_property_graph_->mutex_);
      branch_prop = new DataPropertyBranch_t(property);
      data_property_graph_->container_.insert(branch_prop);
      data_property_graph_->all_branchs_.push_back(branch_prop);
    }

    if(checkRangeAndDomain(branch_from, branch_prop, data_branch))
    {
      branch_from->setSteady_data_properties_name(branch_prop);
      branch_from->setSteady_data_properties_data(data_branch);
      branch_from->data_properties_deduced_.push_back(false);
      branch_from->updated_ = true;
      return true;
    }
    else
      return false;
  }
  return false;
}

bool IndividualGraph::addPropertyInvert(std::string& indiv_from, std::string& property, std::string& indiv_on)
{
  IndividualBranch_t* branch_on = findBranch(indiv_on);
  if(branch_on != nullptr)
  {
    IndividualBranch_t* branch_from = findBranch(indiv_from);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if(branch_from == nullptr)
    {
      ClassBranch_t* test = class_graph_->findBranch(indiv_from);
      if(test != nullptr)
        return false; // TODO ERR

      branch_from = new IndividualBranch_t(indiv_from);
      container_.insert(branch_from);
      individuals_.push_back(branch_from);
    }

    ObjectPropertyBranch_t* branch_prop = object_property_graph_->findBranch(property);
    if(branch_prop == nullptr)
    {
      DataPropertyBranch_t* test = data_property_graph_->findBranch(property);
      if(test != nullptr)
        return false; // TODO ERR

      std::lock_guard<std::shared_timed_mutex> lock_property(object_property_graph_->mutex_);
      branch_prop = new ObjectPropertyBranch_t(property);
      object_property_graph_->container_.insert(branch_prop);
      object_property_graph_->all_branchs_.push_back(branch_prop);
    }

    if(checkRangeAndDomain(branch_from, branch_prop, branch_on))
    {
      int index = -1;
      index = branch_from->ObjectPropertyExistSteady(branch_prop, branch_on);
      if(index == -1)
      {
        branch_from->steady_.object_properties_name_.push_back(branch_prop);
        branch_from->steady_.object_properties_on_.push_back(branch_on);
      }

      index = branch_from->ObjectPropertyExist(branch_prop, branch_on);
      if(index == -1)
      {
        branch_from->object_properties_name_.push_back(branch_prop);
        branch_from->object_properties_on_.push_back(branch_on);
        branch_from->object_properties_deduced_.push_back(false);
        branch_from->object_properties_has_induced_.push_back(Triplet());
      }
      else
        branch_from->object_properties_deduced_[index] = false;

      branch_from->updated_ = true;
      setObjectPropertiesUpdated(branch_from->object_properties_on_);
      return true;
    }
    else
      return false;
  }
  return false;
}

void IndividualGraph::removeLang(std::string& indiv, std::string& lang, std::string& name)
{
  IndividualBranch_t* branch = findBranch(indiv);

  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  lang = lang.substr(1);
  removeFromDictionary(branch->dictionary_, lang, name);
  removeFromDictionary(branch->steady_.dictionary_, lang, name);
  removeFromDictionary(branch->muted_dictionary_, lang, name);
  removeFromDictionary(branch->steady_.muted_dictionary_, lang, name);
}

void IndividualGraph::removeInheritage(std::string& class_base, std::string& class_inherited)
{
  IndividualBranch_t* branch_base = findBranch(class_base);
  ClassBranch_t* branch_inherited = class_graph_->findBranch(class_inherited);

  if(branch_base == nullptr)
    return;
  if(branch_inherited == nullptr)
    return;

  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

  removeFromElemVect(branch_base->steady_.is_a_, branch_inherited);
  removeFromElemVect(branch_base->is_a_, branch_inherited);
  removeFromElemVect(branch_inherited->steady_.individual_childs_, branch_base);
  removeFromElemVect(branch_inherited->individual_childs_, branch_base);

  branch_base->updated_ = true;
  branch_inherited->updated_ = true;
}

bool IndividualGraph::removeProperty(IndividualBranch_t* branch_from, ObjectPropertyBranch_t* property, IndividualBranch_t* branch_on)
{
  bool updated = false;
  for(size_t i = 0; i < branch_from->object_properties_name_.size();)
  {
    if(branch_from->object_properties_name_[i] == property)
    {
      if(branch_from->object_properties_on_[i] == branch_on)
      {
        removePropertyInverse(branch_from, branch_from->object_properties_name_[i], branch_from->object_properties_on_[i]);
        removePropertySymetric(branch_from, branch_from->object_properties_name_[i], branch_from->object_properties_on_[i]);
        removePropertyChain(branch_from, branch_from->object_properties_name_[i], branch_from->object_properties_on_[i]);

        branch_from->object_properties_on_[i]->updated_ = true;
        branch_from->object_properties_name_.erase(branch_from->object_properties_name_.begin() + i);
        branch_from->object_properties_on_.erase(branch_from->object_properties_on_.begin() + i);
        branch_from->object_properties_deduced_.erase(branch_from->object_properties_deduced_.begin() + i);
        branch_from->object_properties_has_induced_.erase(branch_from->object_properties_has_induced_.begin() + i);
        updated = true;
      }
      else
        i++;
    }
    else
      i++;
  }

  for(size_t i = 0; i < branch_from->steady_.object_properties_name_.size();)
  {
    if(branch_from->steady_.object_properties_name_[i] == property)
    {
      if(branch_from->steady_.object_properties_on_[i] == branch_on)
      {
        branch_from->steady_.object_properties_on_[i]->updated_ = true;
        branch_from->steady_.object_properties_name_.erase(branch_from->steady_.object_properties_name_.begin() + i);
        branch_from->steady_.object_properties_on_.erase(branch_from->steady_.object_properties_on_.begin() + i);
        updated = true;
      }
      else
        i++;
    }
    else
      i++;
  }
  if(updated == true)
  {
    setObjectPropertiesUpdated(branch_from->steady_.object_properties_on_);
    setObjectPropertiesUpdated(branch_from->object_properties_on_);
  }

  return true;
}

bool IndividualGraph::removeProperty(std::string& indiv_from, std::string& property, std::string& indiv_on)
{
  IndividualBranch_t* branch_from = findBranch(indiv_from);
  if(branch_from != nullptr)
  {
    IndividualBranch_t* branch_on = findBranch(indiv_on);
    if(branch_on != nullptr)
    {
      ObjectPropertyBranch_t* branch_property = object_property_graph_->findBranch(property);
      if(branch_property != nullptr)
        removeProperty(branch_from, branch_property, branch_on);
    }
  }
  return false;
}

bool IndividualGraph::removeProperty(std::string& indiv_from, std::string& property, std::string& type, std::string& data)
{
  IndividualBranch_t* branch_from = findBranch(indiv_from);
  if(branch_from != nullptr)
  {
    for(size_t i = 0; i < branch_from->data_properties_name_.size();)
    {
      if(branch_from->data_properties_name_[i]->value() == property)
      {
        if((branch_from->data_properties_data_[i].type_ == type) &&
          (branch_from->data_properties_data_[i].value_ == data))
        {
          branch_from->data_properties_name_.erase(branch_from->data_properties_name_.begin() + i);
          branch_from->data_properties_data_.erase(branch_from->data_properties_data_.begin() + i);
          branch_from->data_properties_deduced_.erase(branch_from->data_properties_deduced_.begin() + i);
        }
        else
          i++;
      }
      else
        i++;
    }

    for(size_t i = 0; i < branch_from->steady_.data_properties_name_.size();)
    {
      if(branch_from->steady_.data_properties_name_[i]->value() == property)
      {
        if((branch_from->data_properties_data_[i].type_ == type) &&
          (branch_from->data_properties_data_[i].value_ == data))
        {
          branch_from->steady_.data_properties_name_.erase(branch_from->steady_.data_properties_name_.begin() + i);
          branch_from->steady_.data_properties_data_.erase(branch_from->steady_.data_properties_data_.begin() + i);
        }
        else
          i++;
      }
      else
        i++;
    }

    return true;
  }
  return false;
}

void IndividualGraph::setObjectPropertiesUpdated(std::vector<IndividualBranch_t*> branchs)
{
  for(auto branch : branchs)
    branch->updated_ = true;
}

void IndividualGraph::removePropertyInverse(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  for(ObjectPropertyBranch_t* invert : property->inverses_)
  {
    for(size_t i = 0; i < indiv_on->object_properties_name_.size(); i++)
      if((indiv_on->object_properties_name_[i] == invert) &&
        (indiv_on->object_properties_on_[i] == indiv_from))
        {
          indiv_on->object_properties_name_.erase(indiv_on->object_properties_name_.begin() + i);
          indiv_on->object_properties_on_.erase(indiv_on->object_properties_on_.begin() + i);
          indiv_on->object_properties_deduced_.erase(indiv_on->object_properties_deduced_.begin() + i);
          indiv_on->object_properties_has_induced_.erase(indiv_on->object_properties_has_induced_.begin() + i);
        }

    for(size_t i = 0; i < indiv_on->steady_.object_properties_name_.size(); i++)
      if((indiv_on->steady_.object_properties_name_[i] == invert) &&
        (indiv_on->steady_.object_properties_on_[i] == indiv_from))
        {
          indiv_on->steady_.object_properties_name_.erase(indiv_on->steady_.object_properties_name_.begin() + i);
          indiv_on->steady_.object_properties_on_.erase(indiv_on->steady_.object_properties_on_.begin() + i);
        }
  }
}

void IndividualGraph::removePropertySymetric(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  if(property->properties_.symetric_property_ == true)
  {
    for(size_t i = 0; i < indiv_on->object_properties_name_.size(); i++)
      if((indiv_on->object_properties_name_[i] == property) &&
        (indiv_on->object_properties_on_[i] == indiv_from))
        {
          indiv_on->object_properties_name_.erase(indiv_on->object_properties_name_.begin() + i);
          indiv_on->object_properties_on_.erase(indiv_on->object_properties_on_.begin() + i);
          indiv_on->object_properties_deduced_.erase(indiv_on->object_properties_deduced_.begin() + i);
          indiv_on->object_properties_has_induced_.erase(indiv_on->object_properties_has_induced_.begin() + i);
        }

    for(size_t i = 0; i < indiv_on->steady_.object_properties_name_.size(); i++)
      if((indiv_on->steady_.object_properties_name_[i] == property) &&
        (indiv_on->steady_.object_properties_on_[i] == indiv_from))
        {
          indiv_on->steady_.object_properties_name_.erase(indiv_on->steady_.object_properties_name_.begin() + i);
          indiv_on->steady_.object_properties_on_.erase(indiv_on->steady_.object_properties_on_.begin() + i);
        }
  }
}

void IndividualGraph::removePropertyChain(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  for(size_t k = 0; k < property->chains_.size(); k++)
  {
    std::vector<IndividualBranch_t*> indivs_on = resolveLink(property->chains_[k], indiv_on, 0);
    for(auto indiv : indivs_on)
    {
      ObjectPropertyBranch_t* chained_property = property->chains_[k][property->chains_[k].size() - 1];

      for(size_t i = 0; i < indiv_from->object_properties_name_.size(); i++)
        if(indiv_from->object_properties_name_[i] == chained_property)
          if(indiv_from->object_properties_on_[i] == indiv)
          {
            for(size_t induced = 0; induced < indiv_from->object_properties_has_induced_[i].from_.size(); induced++)
            {
              removeProperty(indiv_from->object_properties_has_induced_[i].from_[induced],
                            indiv_from->object_properties_has_induced_[i].prop_[induced],
                            indiv_from->object_properties_has_induced_[i].on_[induced]);
            }
          }

      removeProperty(indiv_from, chained_property, indiv);
    }
  }
}

std::vector<IndividualBranch_t*> IndividualGraph::resolveLink(std::vector<ObjectPropertyBranch_t*>& chain, IndividualBranch_t* indiv_on, size_t index)
{
  std::vector<IndividualBranch_t*> new_on;
  if(chain.size() > 0)
  {
    if(index != chain.size() - 1)
    {
      for(size_t i = 0; i < indiv_on->object_properties_name_.size(); i++)
      {
        if(indiv_on->object_properties_name_[i] == chain[index])
        {
          std::vector<IndividualBranch_t*> tmp = resolveLink(chain, indiv_on->object_properties_on_[i], index + 1);
          new_on.insert(new_on.end(), tmp.begin(), tmp.end());
        }
      }
    }
    else
      new_on.push_back(indiv_on);
  }
  return new_on;
}

bool IndividualGraph::checkRangeAndDomain(IndividualBranch_t* from, ObjectPropertyBranch_t* prop, IndividualBranch_t* on)
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
    ClassBranch_t* intersection = class_graph_->findIntersection(up_from, domain);
    if(intersection == nullptr)
    {
      std::unordered_set<ClassBranch_t*> disjoints;
      for(auto dom : domain)
        class_graph_->getDisjoint(dom, disjoints);
      intersection = class_graph_->findIntersection(up_from, disjoints);

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
    ClassBranch_t* intersection = class_graph_->findIntersection(up_on, range);
    if(intersection == nullptr)
    {
      std::unordered_set<ClassBranch_t*> disjoints;
      for(auto ran : range)
        class_graph_->getDisjoint(ran, disjoints);
      intersection = class_graph_->findIntersection(up_on, disjoints);

      if(intersection == nullptr)
        from->flags_["range"].push_back(prop->value());
      else
        return false;
    }
  }

  return true;
}

bool IndividualGraph::checkRangeAndDomain(IndividualBranch_t* from, DataPropertyBranch_t* prop, data_t& data)
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
    ClassBranch_t* intersection = class_graph_->findIntersection(up_from, domain);
    if(intersection == nullptr)
    {
      std::unordered_set<ClassBranch_t*> disjoints;
      for(auto dom : domain)
        class_graph_->getDisjoint(dom, disjoints);
      intersection = class_graph_->findIntersection(up_from, disjoints);

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
    std::unordered_set<std::string>::iterator intersection = std::find(range.begin(), range.end(), data.type_);
    if(intersection == range.end())
      return false;
  }

  return true;
}

} // namespace ontologenius
