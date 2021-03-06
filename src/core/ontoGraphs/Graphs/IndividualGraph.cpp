#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include <algorithm>
#include <random>

#include "ontologenius/core/Algorithms/LevenshteinDistance.h"

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

  for(auto indiv : other.individuals_)
    individuals_.push_back(new IndividualBranch_t(indiv->value()));

  container_.load(individuals_);
}

IndividualGraph::~IndividualGraph()
{
  for(auto& individual : individuals_)
    delete individual;

  individuals_.clear();
}

void IndividualGraph::close()
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  container_.load(individuals_);
}

void IndividualGraph::add(const std::string& value, IndividualVectors_t& individual_vector)
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
      // erase because will be pushed again at the end
      break;
    }
  }

  if(me == nullptr)
    me = new IndividualBranch_t(value);

  /**********************
  ** Class assertion
  **********************/
  //for all my classes
  for(auto& is_a : individual_vector.is_a_)
  {
    ClassBranch_t* mother_branch = nullptr;
    getInMap(&mother_branch, is_a.elem, class_graph_->roots_);
    getInMap(&mother_branch, is_a.elem, class_graph_->branchs_);

    //I create my class
    if(mother_branch == nullptr)
    {
      ObjectVectors_t empty_vectors;
      class_graph_->add(is_a.elem, empty_vectors);
      getInMap(&mother_branch, is_a.elem, class_graph_->roots_);
    }

    conditionalPushBack(me->is_a_, ClassElement_t(mother_branch));
    conditionalPushBack(mother_branch->individual_childs_, IndividualElement_t(me));
  }

  /**********************
  ** Object Property assertion
  **********************/
  for(auto& object_relation : individual_vector.object_relations_)
  {
    addObjectProperty(me, object_relation);
    me->object_properties_has_induced_.emplace_back();
  }

  /**********************
  ** Data Property assertion name
  **********************/
  //for all my properties
  for(auto& data_relation : individual_vector.data_relations_)
    addDataProperty(me, data_relation);

  /**********************
  ** Same In Individual
  **********************/
  //for all my inverses
  for(auto& same_as : individual_vector.same_as_)
  {
    bool i_find_my_same = false;

    //is a root my inverse ?
    for(auto& individual : individuals_)
      if(same_as == individual->value())
      {
        conditionalPushBack(me->same_as_, IndividualElement_t(individual));
        conditionalPushBack(individual->same_as_, IndividualElement_t(me, 1.0, true));
        i_find_my_same = true;
      }

    //I create my same
    if(!i_find_my_same)
    {
      auto my_same = new IndividualBranch_t(same_as.elem);
      conditionalPushBack(me->same_as_, IndividualElement_t(my_same));
      conditionalPushBack(my_same->same_as_, IndividualElement_t(me, 1.0, true));
      individuals_.push_back(my_same);
    }
  }

  me->setSteady_dictionary(individual_vector.dictionary_);
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
    for(auto& individual : individuals_)
      if(distinct[distinct_i] == individual->value())
        me = individual;

    // I don't exist ?
    if(me == nullptr)
    {
      me = new IndividualBranch_t(distinct[distinct_i]);
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
        for(auto& individual : individuals_)
          if(distinct[distinct_j] == individual->value())
          {
            conditionalPushBack(me->distinct_, IndividualElement_t(individual));
            i_find_my_distinct = true;
          }

        //I create my distinct
        if(!i_find_my_distinct)
        {
          auto my_distinct = new IndividualBranch_t(distinct[distinct_j]);
          conditionalPushBack(me->distinct_, IndividualElement_t(my_distinct));
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

void IndividualGraph::addObjectProperty(IndividualBranch_t* me, Pair_t<std::string, std::string>& relation)
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

  IndividualBranch_t* indiv_branch = getBranch(relation.second);
  if(indiv_branch == nullptr)
  {
    indiv_branch = new IndividualBranch_t(relation.second);
    individuals_.push_back(indiv_branch);
  }

  me->object_relations_.emplace_back(property_branch, indiv_branch, relation.probability);
}

void IndividualGraph::addDataProperty(IndividualBranch_t* me, Pair_t<std::string, data_t>& relation)
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

  me->data_relations_.emplace_back(property_branch, relation.second, relation.probability);
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
    for(auto& distinct : indiv->distinct_)
      getSameAndClean(distinct.elem, res);
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

void IndividualGraph::getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<std::string>& res, int depth)
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
  std::unordered_set<uint32_t> object_properties = object_property_graph_->getDownIdSafe(property);
  std::unordered_set<uint32_t> data_properties = data_property_graph_->getDownIdSafe(property);

  std::unordered_set<std::string> class_res;
  class_graph_->getRelatedFrom(object_properties, data_properties, class_res);

  std::unordered_set<std::string> res;
  std::lock_guard<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(auto& individual : individuals_)
  {
    for(IndivObjectRelationElement_t& relation : individual->object_relations_)
      for (uint32_t id : object_properties)
        if(relation.first->get() == id)
        {
          getSameAndClean(individual, res);
          break;
        }

    for(IndivDataRelationElement_t& relation : individual->data_relations_)
      for (uint32_t id : data_properties)
        if(relation.first->get() == id)
        {
          getSameAndClean(individual, res);
          break;
        }

    std::unordered_set<std::string> up_set = getUp(individual, 1);
    for(auto& up : up_set)
      if(class_res.find(up) != class_res.end())
      {
        getSameAndClean(individual, res);
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
    for(auto& indiv : individuals_)
      for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
        if(relation.second->get() == id)
          object_property_graph_->getUp(relation.first, res, depth);

  if(res.size() == 0)
  {
    data_t data_img(individual);

    for(auto& indiv : individuals_)
      for(IndivDataRelationElement_t& relation : indiv->data_relations_)
        if(relation.second == data_img)
          data_property_graph_->getUp(relation.first, res, depth);

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
  for(auto& indiv : individuals_)
  {
    for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
      for (uint32_t id : object_properties)
        if(relation.first->get() == id)
          getSameAndClean(relation.second, res);

    for(IndivDataRelationElement_t& relation : indiv->data_relations_)
      for (uint32_t id : data_properties)
        if(relation.first->get() == id)
          res.insert(relation.second.toString());
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
      for(IndivObjectRelationElement_t& relation : it->object_relations_)
      {
        std::unordered_set<IndividualBranch_t*> sames_tmp;
        getSame(relation.second, sames_tmp);
        std::unordered_set<std::string> tmp = set2set(sames_tmp);
        res.insert(tmp.begin(), tmp.end());

        properties[relation.first->value()] = tmp_res.size();
        depths.push_back(0);
        tmp_res.push_back(relation.second->value());
      }

      for(IndivDataRelationElement_t& relation : it->data_relations_)
      {
        res.insert(relation.second.toString());

        properties[relation.first->value()] = tmp_res.size();
        depths.push_back(0);
        tmp_res.push_back(relation.second.toString());
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

  data_t data_img(individual);

  for(auto& indiv : individuals_)
  {
    bool found = false;
    std::unordered_set<uint32_t> took;

    for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
      if(relation.second->value() == individual)
      {
        found = true;
        took.insert(relation.first->get());
      }

    for(IndivDataRelationElement_t& relation : indiv->data_relations_)
    {
      if(relation.second == data_img)
      {
        found = true;
        took.insert(relation.first->get());
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
      getSameAndClean(indiv, res);
  }

  return res;
}

bool IndividualGraph::getRelatedWith(ClassBranch_t* class_branch, const std::string& data, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<uint32_t>& took)
{
  bool res = false;
  if(class_branch != nullptr)
  {
    for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
    {
      if(relation.second->value() == data)
        if(took.find(relation.first->get()) == took.end())
          res = true;
      took.insert(relation.first->get());
    }

    data_t data_img(data);

    for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
    {
      if(relation.second == data_img)
        if(took.find(relation.first->get()) == took.end())
          res = true;
      took.insert(relation.first->get());
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
  size_t pose = param.find(':');
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

  IndividualBranch_t* indiv = container_.find(individual);
  data_t data_img(individual);

  for(auto& indiv_i : individuals_)
  {
    bool found = false;
    bool defined = false;

    if(indiv != nullptr)
    {
      for(IndivObjectRelationElement_t& relation : indiv_i->object_relations_)
        for (uint32_t id : object_properties)
          if(relation.first->get() == id)
          {
            defined = true;
            if(relation.second == indiv)
            {
              found = true;
              break;
            }
          }
    }

    if(defined == false)
      for(IndivDataRelationElement_t& relation : indiv_i->data_relations_)
        for (uint32_t id : data_properties)
          if(relation.first->get() == id)
          {
            if(relation.second == data_img)
            {
              found = true;
              break;
            }
          }


    if((found == false) && (indiv == nullptr))
    {
      std::unordered_set<uint32_t> down_classes = class_graph_->getDownIdSafe(individual);
      std::unordered_set<uint32_t> doNotTake;

      std::unordered_set<ClassBranch_t*> up_set;
      getUpPtr(indiv_i, up_set, 1);
      while(up_set.size() > 0)
      {
        std::unordered_set<ClassBranch_t*> next_step;
        for(auto up : up_set)
          found = found || getFrom(up, object_properties, data_properties, data_img, down_classes, next_step, doNotTake);

        up_set = next_step;
      }
    }

    if(found == true)
      getSameAndClean(indiv_i, res);
  }

  return res;
}

bool IndividualGraph::getFrom(ClassBranch_t* class_branch, std::unordered_set<uint32_t>& object_properties, std::unordered_set<uint32_t>& data_properties, const data_t& data, std::unordered_set<uint32_t>& down_classes, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<uint32_t>& doNotTake)
{
  if(class_branch != nullptr)
  {
    if(doNotTake.find(class_branch->get()) != doNotTake.end())
      return false;

    bool found = false;
    bool defined = false;

    for(ClassObjectRelationElement_t& relation : class_branch->object_relations_)
      for (uint32_t id : object_properties)
        if(relation.first->get() == id)
        {
          defined = true;
          for(uint32_t class_id : down_classes)
            if(relation.second->get() == class_id)
            {
              found = true;
              break;
            }
        }

    if(defined == false)
      for(ClassDataRelationElement_t& relation : class_branch->data_relations_)
        for (uint32_t id : data_properties)
          if(relation.first->get() == id)
          {
            defined = true;
            if(relation.second == data)
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
  size_t pose = param.find(':');
  if(pose != std::string::npos)
  {
    std::string individual = param.substr(0, pose);
    std::string property = param.substr(pose+1);
    return getOn(individual, property);
  }
  return std::unordered_set<std::string>();
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

    for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
      for (uint32_t id : object_properties)
        if(relation.first->get() == id)
          getSameAndClean(relation.second, res);

    for(IndivDataRelationElement_t& relation : indiv->data_relations_)
      for (uint32_t id : data_properties)
        if(relation.first->get() == id)
          res.insert(relation.second.toString());

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
  data_t data_img(second_individual);

  if(indiv != nullptr)
  {
    for(IndivObjectRelationElement_t& relation : indiv->object_relations_)
      if(relation.second->value() == second_individual)
        object_property_graph_->getUp(relation.first, res, depth);

    for(IndivDataRelationElement_t& relation : indiv->data_relations_)
      if(relation.second == data_img)
        data_property_graph_->getUp(relation.first, res, depth);

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

std::unordered_set<std::string> IndividualGraph::getDomainOf(const std::string& individual, int depth)
{
  IndividualBranch_t* branch = container_.find(individual);
  std::unordered_set<ClassBranch_t*> classes;
  getUpPtr(branch, classes, 1);

  std::unordered_set<std::string> res;

  for(auto c : classes)
  {
    std::unordered_set<std::string> tmp = class_graph_->getDomainOf(c, depth);
    res.insert(tmp.begin(), tmp.end());
  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::getRangeOf(const std::string& individual, int depth)
{
  IndividualBranch_t* branch = container_.find(individual);
  std::unordered_set<ClassBranch_t*> classes;
  getUpPtr(branch, classes, 1);

  std::unordered_set<std::string> res;

  for(auto c : classes)
  {
    std::unordered_set<std::string> tmp = class_graph_->getRangeOf(c, depth);
    res.insert(tmp.begin(), tmp.end());
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
      for(auto& is_a : it->is_a_)
        class_graph_->getUp(is_a.elem, res, depth, current_depth);
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
      for(auto& is_a : it->is_a_)
        class_graph_->getUpPtr(is_a.elem, res, depth, current_depth);
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
    for(auto& same : individual->same_as_)
      if(same.elem->mark == false)
        getSame(same.elem, res);
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

void IndividualGraph::getSameAndClean(IndividualBranch_t* individual, std::unordered_set<std::string>& res)
{
  std::unordered_set<IndividualBranch_t*> sames;
  getSame(individual, sames);
  cleanMarks(sames);

  for(IndividualBranch_t* it : sames)
    res.insert(it->value());
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

std::string IndividualGraph::getName(const std::string& value, bool use_default)
{
  std::string res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);

  IndividualBranch_t* branch = container_.find(value);
  if(branch != nullptr)
  {
    if(branch->dictionary_.spoken_.find(language_) != branch->dictionary_.spoken_.end())
    {
      if(branch->dictionary_.spoken_[language_].size())
      {
        std::unordered_set<size_t> tested;
        std::random_device rd;
        std::mt19937 gen(rd());

        size_t dic_size = branch->dictionary_.spoken_[this->language_].size();
        std::uniform_int_distribution<> dis(0, dic_size - 1);

        while(tested.size() < dic_size)
        {
          size_t myIndex = dis(gen);
          std::string word = branch->dictionary_.spoken_[this->language_][myIndex];
          if(word.find('_') == std::string::npos)
          {
            res = word;
            break;
          }
          tested.insert(myIndex);
        }
        if(res == "")
          res = branch->dictionary_.spoken_[this->language_][0];
      }
      else if(use_default)
        res = value;
    }
    else if(use_default)
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
    if(branch->dictionary_.spoken_.find(this->language_) != branch->dictionary_.spoken_.end())
      res = branch->dictionary_.spoken_[this->language_];
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
    if(branch->dictionary_.spoken_.find(this->language_) != branch->dictionary_.spoken_.end())
      res = branch->dictionary_.spoken_[this->language_];
    else
      res.push_back(value);

    if(branch->dictionary_.muted_.find(this->language_) != branch->dictionary_.muted_.end())
    {
      std::vector<std::string> muted = branch->dictionary_.muted_[this->language_];
      res.insert(res.end(), muted.begin(), muted.end());
    }

  }

  return res;
}

std::unordered_set<std::string> IndividualGraph::find(const std::string& value, bool use_default)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(auto& indiv : individuals_)
  {
    if(use_default)
      if(indiv-> value() == value)
        res.insert(indiv->value());

    if(indiv->dictionary_.spoken_.find(language_) != indiv->dictionary_.spoken_.end())
      for(size_t dic_i = 0; dic_i < indiv->dictionary_.spoken_[language_].size(); dic_i++)
        if(indiv->dictionary_.spoken_[language_][dic_i] == value)
          res.insert(indiv->value());

    if(indiv->dictionary_.muted_.find(language_) != indiv->dictionary_.muted_.end())
      for(size_t dic_i = 0; dic_i < indiv->dictionary_.muted_[language_].size(); dic_i++)
        if(indiv->dictionary_.muted_[language_][dic_i] == value)
          res.insert(indiv->value());
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::findSub(const std::string& value, bool use_default)
{
  std::unordered_set<std::string> res;
  std::smatch match;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(auto& indiv : individuals_)
  {
    if(use_default)
    {
      std::regex regex("\\b(" + indiv-> value() + ")([^ ]*)");
      if(std::regex_search(value, match, regex))
        res.insert(indiv->value());
    }

    if(indiv->dictionary_.spoken_.find(language_) != indiv->dictionary_.spoken_.end())
      for(size_t dic_i = 0; dic_i < indiv->dictionary_.spoken_[language_].size(); dic_i++)
      {
        std::regex regex("\\b(" + indiv->dictionary_.spoken_[language_][dic_i] + ")([^ ]*)");
        if(std::regex_search(value, match, regex))
          res.insert(indiv->value());
      }

    if(indiv->dictionary_.muted_.find(language_) != indiv->dictionary_.muted_.end())
      for(size_t dic_i = 0; dic_i < indiv->dictionary_.muted_[language_].size(); dic_i++)
      {
        std::regex regex("\\b(" + indiv->dictionary_.muted_[language_][dic_i] + ")([^ ]*)");
        if(std::regex_search(value, match, regex))
          res.insert(indiv->value());
      }
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::findRegex(const std::string& regex, bool use_default)
{
  std::unordered_set<std::string> res;
  std::regex base_regex(regex);
  std::smatch match;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(auto& indiv : individuals_)
  {
    if(use_default)
    {
      std::string tmp = indiv->value();
      if(std::regex_match(tmp, match, base_regex))
        res.insert(indiv->value());
    }

    if(indiv->dictionary_.spoken_.find(language_) != indiv->dictionary_.spoken_.end())
      for(size_t dic_i = 0; dic_i < indiv->dictionary_.spoken_[language_].size(); dic_i++)
        if(std::regex_match(indiv->dictionary_.spoken_[language_][dic_i], match, base_regex))
          res.insert(indiv->value());

    if(indiv->dictionary_.muted_.find(language_) != indiv->dictionary_.muted_.end())
      for(size_t dic_i = 0; dic_i < indiv->dictionary_.muted_[language_].size(); dic_i++)
        if(std::regex_match(indiv->dictionary_.muted_[language_][dic_i], match, base_regex))
          res.insert(indiv->value());
  }
  return res;
}

std::unordered_set<std::string> IndividualGraph::findFuzzy(const std::string& value, bool use_default, double threshold)
{
  double lower_cost = 100000;
  double tmp_cost = lower_cost;
  std::unordered_set<std::string> res;

  LevenshteinDistance dist;

  std::shared_lock<std::shared_timed_mutex> lock(Graph<IndividualBranch_t>::mutex_);
  for(auto branch : individuals_)
  {
    if(use_default)
      if((tmp_cost = dist.get(branch-> value(), value)) <= lower_cost)
      {
        if(tmp_cost != lower_cost)
        {
          lower_cost = tmp_cost;
          res.clear();
        }
        res.insert(branch->value());
      }

    if(branch->dictionary_.spoken_.find(this->language_) != branch->dictionary_.spoken_.end())
      for(const auto& word : branch->dictionary_.spoken_[this->language_])
        if((tmp_cost = dist.get(word, value)) <= lower_cost)
        {
          if(tmp_cost != lower_cost)
          {
            lower_cost = tmp_cost;
            res.clear();
          }
          res.insert(word);
        }

    if(branch->dictionary_.muted_.find(this->language_) != branch->dictionary_.muted_.end())
      for(const auto& word : branch->dictionary_.muted_[this->language_])
        if((tmp_cost = dist.get(word, value)) <= lower_cost)
        {
          if(tmp_cost != lower_cost)
          {
            lower_cost = tmp_cost;
            res.clear();
          }
          res.insert(word);
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
    auto class_branch = new ClassBranch_t(indiv->value());
    class_branch->mothers_ = std::move(indiv->is_a_);
    class_branch->data_relations_.clear();
    for(auto& data_relation : indiv->data_relations_)
      class_branch->data_relations_.emplace_back(data_relation.first,data_relation.second, data_relation.probability);

    class_graph_->container_.insert(class_branch);
    class_graph_->all_branchs_.push_back(class_branch);
    redirectDeleteIndividual(indiv, class_branch);

    return class_branch;
  }
  return nullptr;
}

IndividualBranch_t* IndividualGraph::createIndividual(const std::string& name)
{
  IndividualBranch_t* indiv = findBranch(name);
  if(indiv == nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    indiv = new IndividualBranch_t(name);
    container_.insert(indiv);
    individuals_.push_back(indiv);
  }
  return indiv;
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

    //erase properties applied to indiv
    size_t indiv_index = 0;
    for(size_t indiv_i = 0; indiv_i < individuals_.size(); indiv_i++)
    {
      if(individuals_[indiv_i] == indiv)
        indiv_index = indiv_i;

      for(size_t i = 0; i < individuals_[indiv_i]->object_relations_.size();)
        if(individuals_[indiv_i]->object_relations_[i].second == indiv)
        {
          individuals_[indiv_i]->object_relations_.erase(individuals_[indiv_i]->object_relations_.begin() + i);
          individuals_[indiv_i]->object_properties_has_induced_.erase(individuals_[indiv_i]->object_properties_has_induced_.begin() + i);
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
          up->childs_.emplace_back(_class);
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

      for(size_t i = 0; i < individuals_[indiv_i]->object_relations_.size();)
        if(individuals_[indiv_i]->object_relations_[i].second == indiv)
        {
          individuals_[indiv_i]->object_relations_.erase(individuals_[indiv_i]->object_relations_.begin() + i);
          individuals_[indiv_i]->object_properties_has_induced_.erase(individuals_[indiv_i]->object_properties_has_induced_.begin() + i);
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

void IndividualGraph::addLang(const std::string& indiv, const std::string& lang, const std::string& name)
{
  IndividualBranch_t* branch = findBranch(indiv);
  if(branch != nullptr)
  {
    auto lang_id = lang.substr(1);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    branch->setSteady_dictionary(lang_id, name);
    branch->updated_ = true;
  }
}

void IndividualGraph::addInheritage(const std::string& indiv, const std::string& class_inherited)
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
    conditionalPushBack(branch->is_a_, ClassElement_t(inherited));
    conditionalPushBack(inherited->individual_childs_, IndividualElement_t(branch));
    branch->updated_ = true;
    inherited->updated_ = true;
  }
}

void IndividualGraph::addInheritageInvert(const std::string& indiv, const std::string& class_inherited)
{
  ClassBranch_t* inherited = class_graph_->findBranch(class_inherited);
  if(inherited != nullptr)
  {
    IndividualBranch_t* branch = createIndividual(indiv);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

    conditionalPushBack(branch->is_a_, ClassElement_t(inherited));
    conditionalPushBack(inherited->individual_childs_, IndividualElement_t(branch));
    branch->updated_ = true;
    inherited->updated_ = true;
  }
}

void IndividualGraph::addInheritageInvertUpgrade(const std::string& indiv, const std::string& class_inherited)
{
  IndividualBranch_t* tmp = findBranch(class_inherited);
  if(tmp != nullptr)
  {
    ClassBranch_t* inherited = upgradeToBranch(tmp);
    IndividualBranch_t* branch = createIndividual(indiv);
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

    conditionalPushBack(branch->is_a_, ClassElement_t(inherited));
    conditionalPushBack(inherited->individual_childs_, IndividualElement_t(branch));
    branch->updated_ = true;
    inherited->updated_ = true;
  }
}

bool IndividualGraph::addProperty(const std::string& indiv_from, const std::string& property, const std::string& indiv_on)
{
  IndividualBranch_t* branch_from = findBranch(indiv_from);
  if(branch_from != nullptr)
  {
    IndividualBranch_t* branch_on = findBranch(indiv_on);
    if(branch_on == nullptr)
    {
      ClassBranch_t* test = class_graph_->findBranch(indiv_on);
      if(test != nullptr)
        return false; // TODO ERR

      branch_on = createIndividual(indiv_on);
    }
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);

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

      index = branch_from->ObjectPropertyExist(branch_prop, branch_on);
      if(index == -1)
      {
        branch_from->object_relations_.emplace_back(IndivObjectRelationElement_t(branch_prop, branch_on));
        branch_from->object_properties_has_induced_.emplace_back();
      }
      else
      {
        branch_from->object_relations_[index].probability = 1.0;
        branch_from->object_relations_[index].infered = false;
      }

      branch_from->updated_ = true;
      setObjectPropertiesUpdated(branch_from->object_relations_);
      return true;
    }
    else
      return false;
  }
  return false;
}

bool IndividualGraph::addProperty(const std::string& indiv_from, const std::string& property, const std::string& type, const std::string& data)
{
  IndividualBranch_t* branch_from = findBranch(indiv_from);
  if(branch_from != nullptr)
  {
    data_t data_branch(type, data);

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
      conditionalPushBack(branch_from->data_relations_, IndivDataRelationElement_t(branch_prop,data_branch));
      branch_from->updated_ = true;
      return true;
    }
    else
      return false;
  }
  return false;
}

bool IndividualGraph::addPropertyInvert(const std::string& indiv_from, const std::string& property, const std::string& indiv_on)
{
  IndividualBranch_t* branch_on = findBranch(indiv_on);
  if(branch_on != nullptr)
  {
    IndividualBranch_t* branch_from = findBranch(indiv_from);
    if(branch_from == nullptr)
    {
      ClassBranch_t* test = class_graph_->findBranch(indiv_from);
      if(test != nullptr)
        return false; // TODO ERR

      branch_from = createIndividual(indiv_from);
    }
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);

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

      index = branch_from->ObjectPropertyExist(branch_prop, branch_on);
      if(index == -1)
      {
        branch_from->object_relations_.emplace_back(IndivObjectRelationElement_t(branch_prop, branch_on));
        branch_from->object_properties_has_induced_.emplace_back();
      }
      else
      {
        branch_from->object_relations_[index].probability = 1.0;
        branch_from->object_relations_[index].infered = false;
      }

      branch_from->updated_ = true;
      setObjectPropertiesUpdated(branch_from->object_relations_);
      return true;
    }
    else
      return false;
  }
  return false;
}

void IndividualGraph::removeLang(const std::string& indiv, const std::string& lang, const std::string& name)
{
  IndividualBranch_t* branch = findBranch(indiv);

  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  auto lang_id = lang.substr(1);
  removeFromDictionary(branch->dictionary_.spoken_, lang_id, name);
  removeFromDictionary(branch->dictionary_.muted_, lang_id, name);
  removeFromDictionary(branch->steady_dictionary_.spoken_, lang_id, name);
  removeFromDictionary(branch->steady_dictionary_.muted_, lang_id, name);
}

void IndividualGraph::removeInheritage(const std::string& class_base, const std::string& class_inherited)
{
  IndividualBranch_t* branch_base = findBranch(class_base);
  ClassBranch_t* branch_inherited = class_graph_->findBranch(class_inherited);

  if(branch_base == nullptr)
    return;
  if(branch_inherited == nullptr)
    return;

  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  std::lock_guard<std::shared_timed_mutex> lock_class(class_graph_->mutex_);

  removeFromElemVect(branch_base->is_a_, branch_inherited);
  removeFromElemVect(branch_inherited->individual_childs_, branch_base);

  branch_base->updated_ = true;
  branch_inherited->updated_ = true;
}

bool IndividualGraph::addSameAs(const std::string& indiv_1, const std::string& indiv_2)
{
  IndividualBranch_t* branch_1 = findBranch(indiv_1);
  IndividualBranch_t* branch_2 = findBranch(indiv_2);

  if((branch_1 == nullptr) && (branch_2 == nullptr))
    return false;

  if(branch_1 == nullptr)
    branch_1 = createIndividual(indiv_1);
  else if(branch_2 == nullptr)
    branch_2 = createIndividual(indiv_2);
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  conditionalPushBack(branch_1->same_as_, IndividualElement_t(branch_2));
  conditionalPushBack(branch_2->same_as_, IndividualElement_t(branch_1));

  return true;
}

bool IndividualGraph::removeSameAs(const std::string& indiv_1, const std::string& indiv_2)
{
  IndividualBranch_t* branch_1 = findBranch(indiv_1);
  IndividualBranch_t* branch_2 = findBranch(indiv_2);

  if((branch_1 == nullptr) || (branch_2 == nullptr))
    return false;

  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  removeFromVect(branch_1->same_as_, IndividualElement_t(branch_2));
  removeFromVect(branch_2->same_as_, IndividualElement_t(branch_1));

  return true;
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removeProperty(IndividualBranch_t* branch_from, ObjectPropertyBranch_t* property, IndividualBranch_t* branch_on)
{
  std::vector<std::pair<std::string, std::string>> explanations;
  bool updated = false;
  bool applied = false;
  std::unordered_set<ObjectPropertyBranch_t*> down_properties;
  object_property_graph_->getDownPtr(property, down_properties);

  for(size_t i = 0; i < branch_from->object_relations_.size();)
  {
    auto object_relation = branch_from->object_relations_[i];
    applied = false;
    for(auto& down : down_properties)
    {
      if(object_relation.first == down)
      {
        if((branch_on == nullptr) || (object_relation.second == branch_on))
        {
          auto exp_inv = removePropertyInverse(branch_from, object_relation.first, object_relation.second);
          auto exp_sym = removePropertySymetric(branch_from, object_relation.first, object_relation.second);
          auto exp_ch  = removePropertyChain(branch_from, object_relation.first, object_relation.second);

          explanations.insert(explanations.end(), exp_inv.begin(), exp_inv.end());
          explanations.insert(explanations.end(), exp_sym.begin(), exp_sym.end());
          explanations.insert(explanations.end(), exp_ch.begin(), exp_ch.end());

          object_relation.second->updated_ = true;
          branch_from->object_relations_.erase(branch_from->object_relations_.begin() + i);
          branch_from->object_properties_has_induced_.erase(branch_from->object_properties_has_induced_.begin() + i);
          updated = true;
          applied = true;
          break;
        }
      }
    }

    if(applied == false)
      i++;
  }
  if(updated == true)
    setObjectPropertiesUpdated(branch_from->object_relations_);

  return explanations;
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removeProperty(const std::string& indiv_from, const std::string& property, const std::string& indiv_on)
{
  IndividualBranch_t* branch_from = findBranch(indiv_from);
  if(branch_from != nullptr)
  {
    if(indiv_on != "_")
    {
      IndividualBranch_t* branch_on = findBranch(indiv_on);
      if(branch_on != nullptr)
      {
        ObjectPropertyBranch_t* branch_property = object_property_graph_->findBranch(property);
        if(branch_property != nullptr)
          return removeProperty(branch_from, branch_property, branch_on);
      }
    }
    else
    {
      ObjectPropertyBranch_t* branch_property = object_property_graph_->findBranch(property);
      if(branch_property != nullptr)
        return removeProperty(branch_from, branch_property, nullptr);
    }
  }
  return std::vector<std::pair<std::string, std::string>>();
}

bool IndividualGraph::removeProperty(const std::string& indiv_from, const std::string& property, const std::string& type, const std::string& data)
{
  IndividualBranch_t* branch_from = findBranch(indiv_from);
  if(branch_from != nullptr)
  {
    for(size_t i = 0; i < branch_from->data_relations_.size();)
    {
      if(branch_from->data_relations_[i].first->value() == property)
      {
        if(((type == "_") || (branch_from->data_relations_[i].second.type_ == type)) &&
          ((data == "_") || (branch_from->data_relations_[i].second.value_ == data)))
          branch_from->data_relations_.erase(branch_from->data_relations_.begin() + i);
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

void IndividualGraph::setObjectPropertiesUpdated(std::vector<IndivObjectRelationElement_t>& relations)
{
  for(auto relation : relations)
    relation.second->updated_ = true;
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removePropertyInverse(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  std::vector<std::pair<std::string, std::string>> explanations;
  for(auto& invert : property->inverses_)
  {
    for(size_t i = 0; i < indiv_on->object_relations_.size(); i++)
      if((indiv_on->object_relations_[i].first == invert.elem) &&
        (indiv_on->object_relations_[i].second == indiv_from))
        {
          explanations.emplace_back("[DEL]" + indiv_on->value() + "|" + indiv_on->object_relations_[i].first->value() + "|" + indiv_on->object_relations_[i].second->value(),
                                     "[DEL]" + indiv_from->value() + "|" + property->value() + "|" + indiv_on->value());
          indiv_on->object_relations_.erase(indiv_on->object_relations_.begin() + i);
          indiv_on->object_properties_has_induced_.erase(indiv_on->object_properties_has_induced_.begin() + i);
        }
  }
  return explanations;
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removePropertySymetric(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
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
          indiv_on->object_relations_.erase(indiv_on->object_relations_.begin() + i);
          indiv_on->object_properties_has_induced_.erase(indiv_on->object_properties_has_induced_.begin() + i);
        }
  }
  return explanations;
}

std::vector<std::pair<std::string, std::string>> IndividualGraph::removePropertyChain(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  std::vector<std::pair<std::string, std::string>> explanations;

  for(size_t i = 0; i < indiv_from->object_relations_.size(); i++)
  {
    if(indiv_from->object_relations_[i].first == property &&
      indiv_from->object_relations_[i].second == indiv_on)
    {
      for(size_t induced = 0; induced < indiv_from->object_properties_has_induced_[i].from_.size(); induced++)
      {
        explanations.emplace_back("[DEL]" + indiv_from->object_properties_has_induced_[i].from_[induced]->value() + "|" +
                                            indiv_from->object_properties_has_induced_[i].prop_[induced]->value() + "|" +
                                            indiv_from->object_properties_has_induced_[i].on_[induced]->value(),
                                   "[DEL]" + indiv_from->value() + "|" + property->value() + "|" + indiv_on->value());

        auto tmp = removeProperty(indiv_from->object_properties_has_induced_[i].from_[induced],
                                  indiv_from->object_properties_has_induced_[i].prop_[induced],
                                  indiv_from->object_properties_has_induced_[i].on_[induced]);
        explanations.insert(explanations.end(), tmp.begin(), tmp.end());
      }
    }
  }

  auto chains = getChains(property);
  for(auto& chain : chains)
  {
    std::vector<IndividualBranch_t*> indivs_on = resolveLink(chain, indiv_on, 0);
    for(auto indiv : indivs_on)
    {
      ObjectPropertyBranch_t* chained_property = chain[chain.size() - 1];

      explanations.emplace_back("[DEL]" + indiv_from->value() + "|" + chained_property->value() + "|" + indiv->value(),
                                 "[DEL]" + indiv_from->value() + "|" + property->value() + "|" + indiv_on->value());
      auto tmp = removeProperty(indiv_from, chained_property, indiv);
      explanations.insert(explanations.end(), tmp.begin(), tmp.end());
    }
  }
  return explanations;
}

std::vector<IndividualBranch_t*> IndividualGraph::resolveLink(std::vector<ObjectPropertyBranch_t*>& chain, IndividualBranch_t* indiv_on, size_t index)
{
  std::vector<IndividualBranch_t*> new_on;
  if(chain.size() > 0)
  {
    if(index != chain.size() - 1)
    {
      for(auto& object_relation : indiv_on->object_relations_)
      {
        if(object_relation.first == chain[index])
        {
          std::vector<IndividualBranch_t*> tmp = resolveLink(chain, object_relation.second, index + 1);
          new_on.insert(new_on.end(), tmp.begin(), tmp.end());
        }
      }
    }
    else
      new_on.push_back(indiv_on);
  }
  return new_on;
}

std::vector<std::vector<ObjectPropertyBranch_t*>> IndividualGraph::getChains(ObjectPropertyBranch_t* base_property)
{
  std::vector<std::vector<ObjectPropertyBranch_t*>> chains;
  chains.insert(chains.end(), base_property->chains_.begin(), base_property->chains_.end());

  for(auto mother : base_property->mothers_)
  {
    auto tmp = getChains(mother.elem);
    chains.insert(chains.end(), tmp.begin(), tmp.end());
  }

  return chains;
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
    if(range.find(data.type_) == range.end())
      return false;
  }

  return true;
}

void IndividualGraph::deepCopy(const IndividualGraph& other)
{
  for(size_t i = 0; i < other.individuals_.size(); i++)
    cpyBranch(other.individuals_[i], individuals_[i]);
}

void IndividualGraph::cpyBranch(IndividualBranch_t* old_branch, IndividualBranch_t* new_branch)
{
  new_branch->mark = old_branch->mark;

  new_branch->nb_updates_ = old_branch->nb_updates_;
  new_branch->updated_ = old_branch->updated_;
  new_branch->flags_ = old_branch->flags_;

  new_branch->dictionary_ = old_branch->dictionary_;
  new_branch->steady_dictionary_ = old_branch->steady_dictionary_;

  for(const auto& is_a : old_branch->is_a_)
    new_branch->is_a_.emplace_back(is_a, class_graph_->container_.find(is_a.elem->value()));

  for(const auto& same : old_branch->same_as_)
    new_branch->same_as_.emplace_back(IndividualElement_t(same, container_.find(same.elem->value())));

  for(const auto& distinct : old_branch->distinct_)
    new_branch->distinct_.emplace_back(IndividualElement_t(distinct, container_.find(distinct.elem->value())));

  for(const auto& relation : old_branch->object_relations_)
  {
    auto prop = object_property_graph_->container_.find(relation.first->value());
    auto on = container_.find(relation.second->value());
    new_branch->object_relations_.emplace_back(IndivObjectRelationElement_t(relation, prop, on));
  }

  for(const auto& relation : old_branch->data_relations_)
  {
    auto prop = data_property_graph_->container_.find(relation.first->value());
    auto data = relation.second;
    new_branch->data_relations_.emplace_back(IndivDataRelationElement_t(relation, prop, data));
  }

  for(const auto& induced : old_branch->object_properties_has_induced_)
  {
    new_branch->object_properties_has_induced_.emplace_back();
    for(size_t i = 0; i < induced.from_.size(); i++)
    {
      auto from = container_.find(induced.from_[i]->value());
      auto prop = object_property_graph_->container_.find(induced.prop_[i]->value());
      auto on = container_.find(induced.on_[i]->value());
      new_branch->object_properties_has_induced_.back().push(from, prop, on);
    }
  }
}

} // namespace ontologenius
