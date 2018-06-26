#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"

IndividualGraph::IndividualGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

IndividualGraph::~IndividualGraph()
{
  for(unsigned int i = 0; i < individuals_.size(); i++)
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
  for(unsigned int i = 0; i < individuals_.size(); i++)
  {
    if(individuals_[i]->value_ == value)
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
  for(unsigned int is_a_i = 0; is_a_i < individual_vector.is_a_.size(); is_a_i++)
  {
    bool i_find_my_is_a_ = false;

    //is a root my class ?
    for(unsigned int root_i = 0; root_i < class_graph_->roots_.size(); root_i++)
      if(individual_vector.is_a_[is_a_i] == class_graph_->roots_[root_i]->value_)
      {
        me->setSteady_is_a(class_graph_->roots_[root_i]);
        i_find_my_is_a_ = true;
      }

    //is a branch my class ?
    for(unsigned int branch_i = 0; branch_i < class_graph_->branchs_.size(); branch_i++)
      if(individual_vector.is_a_[is_a_i] == class_graph_->branchs_[branch_i]->value_)
      {
        me->setSteady_is_a(class_graph_->branchs_[branch_i]);
        i_find_my_is_a_ = true;
      }

    //I create my class
    if(!i_find_my_is_a_)
    {
      ObjectVectors_t empty_vectors;
      class_graph_->add(individual_vector.is_a_[is_a_i], empty_vectors);
      for(unsigned int root_i = 0; root_i < class_graph_->roots_.size(); root_i++)
        if(individual_vector.is_a_[is_a_i] == class_graph_->roots_[root_i]->value_)
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
  for(unsigned int property_i = 0; property_i < individual_vector.object_properties_name_.size(); property_i++)
  {
    bool i_find_my_properties = false;

    //is a root my properties ?
    for(unsigned int root_i = 0; root_i < object_property_graph_->roots_.size(); root_i++)
      if(individual_vector.object_properties_name_[property_i] == object_property_graph_->roots_[root_i]->value_)
      {
        me->setSteady_object_properties_name(object_property_graph_->roots_[root_i]);
        i_find_my_properties = true;
      }

    //is a branch my properties ?
    for(unsigned int branch_i = 0; branch_i < object_property_graph_->branchs_.size(); branch_i++)
      if(individual_vector.object_properties_name_[property_i] == object_property_graph_->branchs_[branch_i]->value_)
      {
        me->setSteady_object_properties_name(object_property_graph_->branchs_[branch_i]);
        i_find_my_properties = true;
      }

    //I create my properties
    if(!i_find_my_properties)
    {
      ObjectPropertyVectors_t empty_vectors;
      object_property_graph_->add(individual_vector.object_properties_name_[property_i], empty_vectors);
      for(unsigned int root_i = 0; root_i < object_property_graph_->roots_.size(); root_i++)
        if(individual_vector.object_properties_name_[property_i] == object_property_graph_->roots_[root_i]->value_)
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
  for(unsigned int properties_on_i = 0; properties_on_i < individual_vector.object_properties_on_.size(); properties_on_i++)
  {
    bool i_find_my_properties_on = false;

    //is a individual exist ?
    for(unsigned int indiv_i = 0; indiv_i < individuals_.size(); indiv_i++)
      if(individual_vector.object_properties_on_[properties_on_i] == individuals_[indiv_i]->value_)
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
  for(unsigned int property_i = 0; property_i < individual_vector.data_properties_name_.size(); property_i++)
  {
    bool i_find_my_properties = false;

    //is a root my properties ?
    for(unsigned int root_i = 0; root_i < data_property_graph_->roots_.size(); root_i++)
      if(individual_vector.data_properties_name_[property_i] == data_property_graph_->roots_[root_i]->value_)
      {
        me->setSteady_data_properties_name(data_property_graph_->roots_[root_i]);
        i_find_my_properties = true;
      }

    //is a branch my properties ?
    for(unsigned int branch_i = 0; branch_i < data_property_graph_->branchs_.size(); branch_i++)
      if(individual_vector.data_properties_name_[property_i] == data_property_graph_->branchs_[branch_i]->value_)
      {
        me->setSteady_data_properties_name(data_property_graph_->branchs_[branch_i]);
        i_find_my_properties = true;
      }

    //I create my properties
    if(!i_find_my_properties)
    {
      DataPropertyVectors_t empty_vectors;
      data_property_graph_->add(individual_vector.data_properties_name_[property_i], empty_vectors);
      for(unsigned int root_i = 0; root_i < data_property_graph_->roots_.size(); root_i++)
        if(individual_vector.data_properties_name_[property_i] == data_property_graph_->roots_[root_i]->value_)
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
  for(unsigned int same_i = 0; same_i < individual_vector.same_as_.size(); same_i++)
  {
    bool i_find_my_same = false;

    //is a root my inverse ?
    for(unsigned int individual_i = 0; individual_i < individuals_.size(); individual_i++)
      if(individual_vector.same_as_[same_i] == individuals_[individual_i]->value_)
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
    me->dictionary_["en"].push_back(me->value_);

  individuals_.push_back(me);
}

void IndividualGraph::add(std::vector<std::string>& distinct)
{
  for(unsigned int distinct_i = 0; distinct_i < distinct.size(); distinct_i++)
  {
    //I need to find myself
    IndividualBranch_t* me = nullptr;
    //Am I created ?
    for(unsigned int individual_i = 0; individual_i < individuals_.size(); individual_i++)
      if(distinct[distinct_i] == individuals_[individual_i]->value_)
        me = individuals_[individual_i];

    // I don't exist ?
    if(me == nullptr)
    {
      me = new struct IndividualBranch_t(distinct[distinct_i]);
      individuals_.push_back(me);
    }

    //for all my distincts ...
    for(unsigned int distinct_j = 0; distinct_j < distinct.size(); distinct_j++)
    {
      //... excepted me
      if(distinct_i != distinct_j)
      {
        bool i_find_my_distinct = false;

        //is my distinct created ?
        for(unsigned int individual_i = 0; individual_i < individuals_.size(); individual_i++)
          if(distinct[distinct_j] == individuals_[individual_i]->value_)
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

std::set<std::string> IndividualGraph::getSame(const std::string& individual)
{
  std::set<std::string> res;
  std::set<IndividualBranch_t*> tmp = getSame(container_.find(individual));
  cleanMarks(tmp);
  for(std::set<IndividualBranch_t*>::iterator it = tmp.begin(); it != tmp.end(); ++it)
    res.insert((*it)->value_);
  return res;
}

std::set<std::string> IndividualGraph::getDistincts(const std::string& individual)
{
  std::set<std::string> res;
  IndividualBranch_t* indiv = container_.find(individual);
  if(indiv != nullptr)
    for(size_t i = 0; i < indiv->distinct_.size(); i++)
    {
      std::set<IndividualBranch_t*> sames = getSame(indiv->distinct_[i]);
      std::set<std::string> tmp = set2set(sames);
      cleanMarks(sames);
      res.insert(tmp.begin(), tmp.end());
    }
  return res;
}

std::set<std::string> IndividualGraph::getRelationFrom(const std::string& individual, int depth)
{
  std::set<std::string> res;
  IndividualBranch_t* indiv = container_.find(individual);
  if(indiv != nullptr)
  {
    std::set<IndividualBranch_t*> sames = getSame(indiv);
    cleanMarks(sames);
    for(std::set<IndividualBranch_t*>::iterator it = sames.begin(); it != sames.end(); ++it)
    {
      for(size_t i = 0; i < (*it)->object_properties_name_.size(); i++)
      {
        std::set<std::string> tmp = object_property_graph_->getUp((*it)->object_properties_name_[i], depth);
        res.insert(tmp.begin(), tmp.end());
      }

      for(size_t i = 0; i < (*it)->data_properties_name_.size(); i++)
      {
        std::set<std::string> tmp = data_property_graph_->getUp((*it)->data_properties_name_[i], depth);
        res.insert(tmp.begin(), tmp.end());
      }
    }
  }
  return res;
}

std::set<std::string> IndividualGraph::getRelatedFrom(const std::string& property)
{
  std::set<std::string> object_properties = object_property_graph_->getDown(property);
  std::set<std::string> data_properties = data_property_graph_->getDown(property);
  std::set<std::string>::iterator it;

  std::set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_name_.size(); prop_i++)
      for (it = object_properties.begin(); it != object_properties.end(); ++it)
        if(individuals_[i]->object_properties_name_[prop_i]->value_ == (*it))
        {
          std::set<IndividualBranch_t*> sames = getSame(individuals_[i]);
          std::set<std::string> tmp = set2set(sames);
          cleanMarks(sames);
          res.insert(tmp.begin(), tmp.end());
        }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_name_.size(); prop_i++)
      for (it = data_properties.begin(); it != data_properties.end(); ++it)
        if(individuals_[i]->data_properties_name_[prop_i]->value_ == (*it))
        {
          std::set<IndividualBranch_t*> sames = getSame(individuals_[i]);
          std::set<std::string> tmp = set2set(sames);
          cleanMarks(sames);
          res.insert(tmp.begin(), tmp.end());
        }
  }
  return res;
}

std::set<std::string> IndividualGraph::getRelationOn(const std::string& individual, int depth)
{
  std::set<std::string> res;
  std::set<std::string> same = getSame(individual);
  for(std::set<std::string>::iterator it = same.begin(); it != same.end(); ++it)
    for(size_t i = 0; i < individuals_.size(); i++)
      for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_on_.size(); prop_i++)
        if(individuals_[i]->object_properties_on_[prop_i]->value_ == (*it))
        {
          std::set<std::string> tmp = object_property_graph_->getUp(individuals_[i]->object_properties_name_[prop_i], depth);
          res.insert(tmp.begin(), tmp.end());
        }

  if(res.size() == 0)
    for(size_t i = 0; i < individuals_.size(); i++)
      for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_data_.size(); prop_i++)
        if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
        {
          std::set<std::string> tmp = data_property_graph_->getUp(individuals_[i]->data_properties_name_[prop_i], depth);
          res.insert(tmp.begin(), tmp.end());
        }

  return res;
}

std::set<std::string> IndividualGraph::getRelatedOn(const std::string& property)
{
  std::set<std::string> object_properties = object_property_graph_->getDown(property);
  std::set<std::string> data_properties = data_property_graph_->getDown(property);
  std::set<std::string>::iterator it;

  std::set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_name_.size(); prop_i++)
      for (it = object_properties.begin(); it != object_properties.end(); ++it)
        if(individuals_[i]->object_properties_name_[prop_i]->value_ == (*it))
        {
          std::set<IndividualBranch_t*> sames = getSame(individuals_[i]->object_properties_on_[prop_i]);
          std::set<std::string> tmp = set2set(sames);
          cleanMarks(sames);
          res.insert(tmp.begin(), tmp.end());
        }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_name_.size(); prop_i++)
      for (it = data_properties.begin(); it != data_properties.end(); ++it)
        if(individuals_[i]->data_properties_name_[prop_i]->value_ == (*it))
          res.insert(individuals_[i]->data_properties_data_[prop_i].toString());
  }

  return res;
}

std::set<std::string> IndividualGraph::getRelationWith(const std::string& individual)
{
  std::set<std::string> res;
  IndividualBranch_t* indiv = container_.find(individual);
  if(indiv != nullptr)
  {
    std::set<IndividualBranch_t*> sames = getSame(indiv);
    cleanMarks(sames);
    for(std::set<IndividualBranch_t*>::iterator it = sames.begin(); it != sames.end(); ++it)
    {
      for(size_t i = 0; i < (*it)->object_properties_on_.size(); i++)
      {
        std::set<std::string> tmp = set2set(getSame((*it)->object_properties_on_[i]));
        res.insert(tmp.begin(), tmp.end());
      }
      for(size_t i = 0; i < (*it)->data_properties_data_.size(); i++)
        res.insert((*it)->data_properties_data_[i].toString());
    }
  }
  return res;
}

std::set<std::string> IndividualGraph::getRelatedWith(const std::string& individual)
{
  std::set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_on_.size(); prop_i++)
      if(individuals_[i]->object_properties_on_[prop_i]->value_ == individual)
      {
        std::set<IndividualBranch_t*> sames = getSame(individuals_[i]);
        std::set<std::string> tmp = set2set(sames);
        cleanMarks(sames);
        res.insert(tmp.begin(), tmp.end());
      }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_data_.size(); prop_i++)
      if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
      {
        std::set<IndividualBranch_t*> sames = getSame(individuals_[i]);
        std::set<std::string> tmp = set2set(sames);
        cleanMarks(sames);
        res.insert(tmp.begin(), tmp.end());
      }
  }

  return res;
}

std::set<std::string> IndividualGraph::getFrom(const std::string& param)
{
  std::set<std::string> res;
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

std::set<std::string> IndividualGraph::getFrom(const std::string& individual, const std::string& property)
{
  std::set<std::string> object_properties = object_property_graph_->getDown(property);
  std::set<std::string> data_properties = data_property_graph_->getDown(property);
  std::set<std::string>::iterator it;

  std::set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_on_.size(); prop_i++)
      if(individuals_[i]->object_properties_on_[prop_i]->value_ == individual)
        for (it = object_properties.begin(); it != object_properties.end(); ++it)
          if(individuals_[i]->object_properties_name_[prop_i]->value_ == (*it))
          {
            std::set<IndividualBranch_t*> sames = getSame(individuals_[i]);
            std::set<std::string> tmp = set2set(sames);
            cleanMarks(sames);
            res.insert(tmp.begin(), tmp.end());
          }

    for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_data_.size(); prop_i++)
      if(individuals_[i]->data_properties_data_[prop_i].value_ == individual)
        for (it = data_properties.begin(); it != data_properties.end(); ++it)
          if(individuals_[i]->data_properties_name_[prop_i]->value_ == (*it))
          {
            std::set<IndividualBranch_t*> sames = getSame(individuals_[i]);
            std::set<std::string> tmp = set2set(sames);
            cleanMarks(sames);
            res.insert(tmp.begin(), tmp.end());
          }
  }

  return res;
}

std::set<std::string> IndividualGraph::getOn(const std::string& param)
{
  std::set<std::string> res;
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

std::set<std::string> IndividualGraph::getOn(const std::string& individual, const std::string& property)
{
  std::set<std::string> object_properties = object_property_graph_->getDown(property);
  std::set<std::string> data_properties = data_property_graph_->getDown(property);
  std::set<std::string>::iterator it;

  std::set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
    if(individuals_[i]->value_ == individual)
    {
      for(size_t prop_i = 0; prop_i < individuals_[i]->object_properties_name_.size(); prop_i++)
        for (it = object_properties.begin(); it != object_properties.end(); ++it)
          if(individuals_[i]->object_properties_name_[prop_i]->value_ == (*it))
          {
            std::set<IndividualBranch_t*> sames = getSame(individuals_[i]->object_properties_on_[prop_i]);
            std::set<std::string> tmp = set2set(sames);
            cleanMarks(sames);
            res.insert(tmp.begin(), tmp.end());
          }

      for(size_t prop_i = 0; prop_i < individuals_[i]->data_properties_name_.size(); prop_i++)
        for (it = data_properties.begin(); it != data_properties.end(); ++it)
          if(individuals_[i]->data_properties_name_[prop_i]->value_ == (*it))
            res.insert(individuals_[i]->data_properties_data_[prop_i].toString());
    }

  return res;
}

std::set<std::string> IndividualGraph::getWith(const std::string& param, int depth)
{
  std::set<std::string> res;
  size_t pose = param.find(":");
  if(pose != std::string::npos)
  {
    std::string first_individual = param.substr(0, pose);
    std::string second_individual = param.substr(pose+1);
    return getWith(first_individual, second_individual, depth);
  }
  return res;
}

std::set<std::string> IndividualGraph::getWith(const std::string& first_individual, const std::string& second_individual, int depth)
{
  std::set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
    if(individuals_[i]->value_ == first_individual)
    {
      for(size_t indiv_i = 0; indiv_i < individuals_[i]->object_properties_on_.size(); indiv_i++)
        if(individuals_[i]->object_properties_on_[indiv_i]->value_ == second_individual)
        {
          std::set<std::string> props = object_property_graph_->getUp(individuals_[i]->object_properties_name_[indiv_i], depth);
          res.insert(props.begin(), props.end());
        }

      for(size_t indiv_i = 0; indiv_i < individuals_[i]->data_properties_data_.size(); indiv_i++)
        if(individuals_[i]->data_properties_data_[indiv_i].value_ == second_individual)
        {
          std::set<std::string> props = data_property_graph_->getUp(individuals_[i]->data_properties_name_[indiv_i], depth);
          res.insert(props.begin(), props.end());
        }
    }
  return res;
}

std::set<std::string> IndividualGraph::getUp(IndividualBranch_t* indiv, int depth, unsigned int current_depth)
{
  current_depth++;
  std::set<std::string> res;
  if(indiv != nullptr)
  {
    std::set<IndividualBranch_t*> sames = getSame(indiv);
    cleanMarks(sames);
    for(std::set<IndividualBranch_t*>::iterator it = sames.begin(); it != sames.end(); ++it)
      for(size_t i = 0; i < (*it)->is_a_.size(); i++)
      {
        std::set<std::string> tmp = class_graph_->getUp((*it)->is_a_[i], depth, current_depth);
        if(tmp.size())
          res.insert(tmp.begin(), tmp.end());
      }
  }
  return res;
}

std::set<std::string> IndividualGraph::getUp(const std::string& individual, int depth)
{
  IndividualBranch_t* indiv = container_.find(individual);
  return getUp(indiv, depth);
}

std::set<IndividualBranch_t*> IndividualGraph::getSame(IndividualBranch_t* individual)
{
  std::set<IndividualBranch_t*> res;
  if(individual != nullptr)
  {
    res.insert(individual);
    individual->mark = true;
    for(size_t i = 0; i < individual->same_as_.size(); i++)
    {
      if(individual->same_as_[i]->mark == false)
      {
        std::set<IndividualBranch_t*> tmp = getSame(individual->same_as_[i]);
        if(tmp.size())
          res.insert(tmp.begin(), tmp.end());
      }
    }
  }
  return res;
}

std::set<std::string> IndividualGraph::select(const std::set<std::string>& on, const std::string& class_selector)
{
  std::set<std::string> res;
  for(std::set<std::string>::iterator it = on.begin(); it != on.end(); ++it)
  {
    std::set<std::string> tmp = getUp(*it);
    if(tmp.find(class_selector) != tmp.end())
      res.insert(*it);
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
        res = branch->dictionary_[language_][0];
      else
        res = value;
    else
      res = value;
  }

  return res;
}

std::set<std::string> IndividualGraph::find(const std::string& value)
{
  std::set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    if(individuals_[i]->dictionary_.find(language_) != individuals_[i]->dictionary_.end())
      for(size_t dic_i = 0; dic_i < individuals_[i]->dictionary_[language_].size(); dic_i++)
        if(individuals_[i]->dictionary_[language_][dic_i] == value)
          res.insert(individuals_[i]->value_);
  }
  return res;
}

std::set<std::string> IndividualGraph::getType(const std::string& class_selector)
{
  std::set<std::string> res;
  for(size_t i = 0; i < individuals_.size(); i++)
  {
    std::set<std::string> tmp = getUp(individuals_[i]);
    if(tmp.find(class_selector) != tmp.end())
      res.insert(individuals_[i]->value_);
  }
  return res;
}

void IndividualGraph::cleanMarks(std::set<IndividualBranch_t*>& indSet)
{
  for(std::set<IndividualBranch_t*>::iterator it = indSet.begin(); it != indSet.end(); ++it)
    (*it)->mark = false;
}

std::set<std::string> IndividualGraph::set2set(std::set<IndividualBranch_t*> indSet, bool clean)
{
  std::set<std::string> res;
  for(std::set<IndividualBranch_t*>::iterator it = indSet.begin(); it != indSet.end(); ++it)
  {
    if(clean)
      (*it)->mark = false;
    res.insert((*it)->value_);
  }
  return res;
}
