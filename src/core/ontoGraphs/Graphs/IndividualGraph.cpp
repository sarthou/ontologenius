#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

#include <random>

IndividualGraph::IndividualGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

IndividualGraph::IndividualGraph(const IndividualGraph& base) : Graph<IndividualBranch_t>(base)
{
  for(IndividualBranch_t* b : base.individuals_)
  {
    IndividualBranch_t* tmp = new IndividualBranch_t();
    *tmp = *b;
    individuals_.push_back(b);
  }
}

IndividualGraph::~IndividualGraph()
{
  for(size_t i = 0; i < individuals_.size(); i++)
    delete individuals_[i];

  individuals_.clear();
}

void IndividualGraph::close()
{
  container_.load(individuals_);
}

void IndividualGraph::add(std::string value, IndividualVectors_t& individual_vector)
{
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
    for(size_t root_i = 0; root_i < class_graph_->roots_.size(); root_i++)
      if(individual_vector.is_a_[is_a_i] == class_graph_->roots_[root_i]->value())
      {
        me->setSteady_is_a(class_graph_->roots_[root_i]);
        i_find_my_is_a_ = true;
      }

    //is a branch my class ?
    for(size_t branch_i = 0; branch_i < class_graph_->branchs_.size(); branch_i++)
      if(individual_vector.is_a_[is_a_i] == class_graph_->branchs_[branch_i]->value())
      {
        me->setSteady_is_a(class_graph_->branchs_[branch_i]);
        i_find_my_is_a_ = true;
      }

    //I create my class
    if(!i_find_my_is_a_)
    {
      ObjectVectors_t empty_vectors;
      class_graph_->add(individual_vector.is_a_[is_a_i], empty_vectors);
      for(size_t root_i = 0; root_i < class_graph_->roots_.size(); root_i++)
        if(individual_vector.is_a_[is_a_i] == class_graph_->roots_[root_i]->value())
        {
          me->setSteady_is_a(class_graph_->roots_[root_i]);
          i_find_my_is_a_ = true;
        }
    }
  }

  /**********************
  ** Object Property assertion name
  **********************/
  //for all my properties
  for(size_t property_i = 0; property_i < individual_vector.object_properties_name_.size(); property_i++)
  {
    bool i_find_my_properties = false;

    //is a root my properties ?
    for(size_t root_i = 0; root_i < object_property_graph_->roots_.size(); root_i++)
      if(individual_vector.object_properties_name_[property_i] == object_property_graph_->roots_[root_i]->value())
      {
        me->setSteady_object_properties_name(object_property_graph_->roots_[root_i]);
        i_find_my_properties = true;
      }

    //is a branch my properties ?
    for(size_t branch_i = 0; branch_i < object_property_graph_->branchs_.size(); branch_i++)
      if(individual_vector.object_properties_name_[property_i] == object_property_graph_->branchs_[branch_i]->value())
      {
        me->setSteady_object_properties_name(object_property_graph_->branchs_[branch_i]);
        i_find_my_properties = true;
      }

    //I create my properties
    if(!i_find_my_properties)
    {
      ObjectPropertyVectors_t empty_vectors;
      object_property_graph_->add(individual_vector.object_properties_name_[property_i], empty_vectors);
      for(size_t root_i = 0; root_i < object_property_graph_->roots_.size(); root_i++)
        if(individual_vector.object_properties_name_[property_i] == object_property_graph_->roots_[root_i]->value())
        {
          me->setSteady_object_properties_name(object_property_graph_->roots_[root_i]);
          i_find_my_properties = true;
        }
    }
  }

  /**********************
  ** Object Property assertion on indiv
  **********************/
  //for all my individuals
  for(size_t properties_on_i = 0; properties_on_i < individual_vector.object_properties_on_.size(); properties_on_i++)
  {
    bool i_find_my_properties_on = false;

    //is a individual exist ?
    for(size_t indiv_i = 0; indiv_i < individuals_.size(); indiv_i++)
      if(individual_vector.object_properties_on_[properties_on_i] == individuals_[indiv_i]->value())
      {
        me->setSteady_object_properties_on(individuals_[indiv_i]);
        i_find_my_properties_on = true;
      }

    //I create my individual
    if(!i_find_my_properties_on)
    {
      IndividualBranch_t* tmp = new IndividualBranch_t(individual_vector.object_properties_on_[properties_on_i]);
      individuals_.push_back(tmp);
      me->setSteady_object_properties_on(tmp);
    }
  }

  /**********************
  ** Data Property assertion name
  **********************/
  //for all my properties
  for(size_t property_i = 0; property_i < individual_vector.data_properties_name_.size(); property_i++)
  {
    bool i_find_my_properties = false;

    //is a root my properties ?
    for(size_t root_i = 0; root_i < data_property_graph_->roots_.size(); root_i++)
      if(individual_vector.data_properties_name_[property_i] == data_property_graph_->roots_[root_i]->value())
      {
        me->setSteady_data_properties_name(data_property_graph_->roots_[root_i]);
        i_find_my_properties = true;
      }

    //is a branch my properties ?
    for(size_t branch_i = 0; branch_i < data_property_graph_->branchs_.size(); branch_i++)
      if(individual_vector.data_properties_name_[property_i] == data_property_graph_->branchs_[branch_i]->value())
      {
        me->setSteady_data_properties_name(data_property_graph_->branchs_[branch_i]);
        i_find_my_properties = true;
      }

    //I create my properties
    if(!i_find_my_properties)
    {
      DataPropertyVectors_t empty_vectors;
      data_property_graph_->add(individual_vector.data_properties_name_[property_i], empty_vectors);
      for(size_t root_i = 0; root_i < data_property_graph_->roots_.size(); root_i++)
        if(individual_vector.data_properties_name_[property_i] == data_property_graph_->roots_[root_i]->value())
        {
          me->setSteady_data_properties_name(data_property_graph_->roots_[root_i]);
          i_find_my_properties = true;
        }
    }

    data_t data;
    data.value_ = individual_vector.data_properties_value_[property_i];
    data.type_ = individual_vector.data_properties_type_[property_i];
    me->setSteady_data_properties_data(data);
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

  individuals_.push_back(me);
}

void IndividualGraph::add(std::vector<std::string>& distinct)
{
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

std::unordered_set<std::string> IndividualGraph::getSame(const std::string& individual)
{
  return getSameAndClean(container_.find(individual));
}

std::unordered_set<std::string> IndividualGraph::getDistincts(const std::string& individual)
{
  std::unordered_set<std::string> res;
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
    for(size_t i = 0; i < class_branch->object_properties_name_.size(); i++)
      object_property_graph_->getUp(class_branch->object_properties_name_[i], res, depth);

    for(size_t i = 0; i < class_branch->data_properties_name_.size(); i++)
      data_property_graph_->getUp(class_branch->data_properties_name_[i], res, depth);
  }
}

std::unordered_set<std::string> IndividualGraph::getRelatedFrom(const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_name_.size(); prop_i++)
      for (uint32_t id : object_properties)
        if(individuals_[i]->object_properties_name_[prop_i]->get() == id)
        {
          std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
          res.insert(tmp.begin(), tmp.end());
        }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_name_.size(); prop_i++)
      for (uint32_t id : data_properties)
        if(individuals_[i]->data_properties_name_[prop_i]->get() == id)
        {
          std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
          res.insert(tmp.begin(), tmp.end());
        }
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelationOn(const std::string& individual, int depth)
{
  std::unordered_set<std::string> res;
  std::unordered_set<uint32_t> same = getSameId(individual);
  for(uint32_t id : same)
    for(size_t i = 0; i < individuals_.size(); i++)
      for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_on_.size(); prop_i++)
        if(individuals_[i]->object_properties_on_[prop_i]->get() == id)
          object_property_graph_->getUp(individuals_[i]->object_properties_name_[prop_i], res, depth);

  if(res.size() == 0)
    for(size_t i = 0; i < individuals_.size(); i++)
      for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_data_.size(); prop_i++)
        if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
          data_property_graph_->getUp(individuals_[i]->data_properties_name_[prop_i], res, depth);

  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelatedOn(const std::string& property)
{
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
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

  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelationWith(const std::string& individual)
{
  std::unordered_set<std::string> res;
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
      }
      for(size_t i = 0; i < it->data_properties_data_.size(); i++)
        res.insert(it->data_properties_data_[i].toString());
    }
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getRelatedWith(const std::string& individual)
{
  std::unordered_set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_on_.size(); prop_i++)
      if(individuals_[i]->object_properties_on_[prop_i]->value() == individual)
      {
        std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
        res.insert(tmp.begin(), tmp.end());
      }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_data_.size(); prop_i++)
      if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
      {
        std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
        res.insert(tmp.begin(), tmp.end());
      }
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
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_on_.size(); prop_i++)
      if(individuals_[i]->object_properties_on_[prop_i]->value() == individual)
        for (uint32_t id : object_properties)
          if(individuals_[i]->object_properties_name_[prop_i]->get() == id)
          {
            std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
            res.insert(tmp.begin(), tmp.end());
          }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_data_.size(); prop_i++)
      if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
        for (uint32_t id : data_properties)
          if(individuals_[i]->data_properties_name_[prop_i]->get() == id)
          {
            std::unordered_set<std::string> tmp = getSameAndClean(individuals_[i]);
            res.insert(tmp.begin(), tmp.end());
          }
  }

  return res;
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
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownId(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownId(property);

  std::unordered_set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
    if(individuals_[i]->value() == individual)
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
  for(size_t i = 0; i < individuals_.size(); i++)
    if(individuals_[i]->value() == first_individual)
    {
      for(size_t indiv_i = 0; indiv_i < individuals_[i]->object_properties_on_.size(); indiv_i++)
        if(individuals_[i]->object_properties_on_[indiv_i]->value() == second_individual)
          object_property_graph_->getUp(individuals_[i]->object_properties_name_[indiv_i], res, depth);

      for(size_t indiv_i = 0; indiv_i < individuals_[i]->data_properties_data_.size(); indiv_i++)
        if(individuals_[i]->data_properties_data_[indiv_i].value_ == second_individual)
          data_property_graph_->getUp(individuals_[i]->data_properties_name_[indiv_i], res, depth);
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
        class_graph_->getUp(it->is_a_[i], res, depth, current_depth);
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getUp(const std::string& individual, int depth)
{
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
        class_graph_->getUpPtr(it->is_a_[i], res, depth, current_depth);
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
  for(const std::string& it : on)
  {
    std::unordered_set<std::string> tmp = getUp(it);
    if(tmp.find(class_selector) != tmp.end())
      res.insert(it);
  }
  return res;
}

std::string IndividualGraph::getName(const std::string& value)
{
  std::string res;

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

std::unordered_set<std::string> IndividualGraph::find(const std::string& value)
{
  std::unordered_set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    if(individuals_[i]->dictionary_.find(language_) != individuals_[i]->dictionary_.end())
      for(size_t dic_i = 0; dic_i < individuals_[i]->dictionary_[language_].size(); dic_i++)
        if(individuals_[i]->dictionary_[language_][dic_i] == value)
          res.insert(individuals_[i]->value());
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::getType(const std::string& class_selector)
{
  std::unordered_set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    std::unordered_set<std::string> tmp = getUp(individuals_[i]);
    if(tmp.find(class_selector) != tmp.end())
      res.insert(individuals_[i]->value());
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
