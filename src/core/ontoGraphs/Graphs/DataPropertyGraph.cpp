#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

#include <iostream>

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"

namespace ontologenius {

void DataPropertyGraph::add(std::string value, DataPropertyVectors_t& property_vectors)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);
  /**********************
  ** Mothers
  **********************/
  DataPropertyBranch_t* me = nullptr;
  //am I a created mother ?
  amIA(&me, tmp_mothers_, value);

  //am I a created branch ?
  amIA(&me, branchs_, value);

  //am I a created root ?
  amIA(&me, roots_, value);

  //am I created ?
  if(me == nullptr)
  {
    me = new DataPropertyBranch_t(value);
  }

  me->nb_mothers_ += property_vectors.mothers_.size();

  //am I a root ?
  if(me->nb_mothers_ == 0)
    roots_[me->value()] = me;
  else
  {
    //for all my mothers
    for(auto& mother : property_vectors.mothers_)
    {
      DataPropertyBranch_t* mother_branch = nullptr;
      getInMap(&mother_branch, mother.elem, roots_);
      getInMap(&mother_branch, mother.elem, branchs_);
      getInMap(&mother_branch, mother.elem, tmp_mothers_);
      if(mother_branch == nullptr)
      {
        mother_branch = new struct DataPropertyBranch_t(mother.elem);
        tmp_mothers_[mother_branch->value()] = mother_branch;
      }

      conditionalPushBack(mother_branch->childs_, DataPropertyElement_t(me, mother.probability, true));
      conditionalPushBack(me->mothers_, DataPropertyElement_t(mother_branch, mother.probability));
    }

    //but i am also a branch
    branchs_[me->value()] = me;
  }

  /**********************
  ** Disjoints
  **********************/
  //for all my disjoints
  for(auto& disjoint : property_vectors.disjoints_)
  {
    DataPropertyBranch_t* disjoint_branch = nullptr;
    getInMap(&disjoint_branch, disjoint.elem, roots_);
    getInMap(&disjoint_branch, disjoint.elem, branchs_);
    getInMap(&disjoint_branch, disjoint.elem, tmp_mothers_);

    //I create my disjoint
    if(disjoint_branch == nullptr)
    {
      disjoint_branch = new struct DataPropertyBranch_t(disjoint.elem);
      tmp_mothers_[disjoint_branch->value()] = disjoint_branch; //I put my disjoint as tmp_mother
    }
    conditionalPushBack(me->disjoints_, DataPropertyElement_t(disjoint_branch, disjoint.probability));
    conditionalPushBack(disjoint_branch->disjoints_, DataPropertyElement_t(me, disjoint.probability, true));
  }

  /**********************
  ** Domains
  **********************/
  //for all my domains
  for(auto& domain : property_vectors.domains_)
  {
    ClassBranch_t* domain_branch = nullptr;
    getInMap(&domain_branch, domain.elem, class_graph_->roots_);
    getInMap(&domain_branch, domain.elem, class_graph_->branchs_);
    getInMap(&domain_branch, domain.elem, class_graph_->tmp_mothers_);

    //I create my domain
    if(domain_branch == nullptr)
    {
      ObjectVectors_t empty_vectors;
      class_graph_->add(domain.elem, empty_vectors);
      getInMap(&domain_branch, domain.elem, class_graph_->roots_);
    }
    conditionalPushBack(me->domains_, ClassElement_t(domain_branch, domain.probability));
  }

  /**********************
  ** Ranges
  **********************/
  //for all my ranges
  for(size_t ranges_i = 0; ranges_i < property_vectors.ranges_.size(); ranges_i++)
  {
    data_t data;
    data.set(property_vectors.ranges_[ranges_i]);
    conditionalPushBack(me->ranges_, data); // FIXME
  }

  /**********************
  ** Language and properties
  **********************/
  me->properties_ = property_vectors.properties_;
  me->setSteady_dictionary(property_vectors.dictionary_);
  if(me->dictionary_.spoken_.find("en") == me->dictionary_.spoken_.end())
    me->dictionary_.spoken_["en"].push_back(me->value());
  me->setSteady_muted_dictionary(property_vectors.muted_dictionary_);

  mitigate(me);
}

void DataPropertyGraph::add(std::vector<std::string>& disjoints)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  for(size_t disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
  {
    //I need to find myself
    DataPropertyBranch_t* me = nullptr;
    //Am I a root ?
    amIA(&me, roots_, disjoints[disjoints_i], false);

    //Am I a branch ?
    amIA(&me, branchs_, disjoints[disjoints_i], false);

    //Am I a tmp_mother ?
    amIA(&me, tmp_mothers_, disjoints[disjoints_i], false);

    // I don't exist ? so I will be a tmp_mother
    if(me == nullptr)
    {
      me = new struct DataPropertyBranch_t(disjoints[disjoints_i]);
      tmp_mothers_[me->value()] = me;
    }

    //for all my disjoints ...
    for(size_t disjoints_j = 0; disjoints_j < disjoints.size(); disjoints_j++)
    {
      //... excepted me
      if(disjoints_i != disjoints_j)
      {
        DataPropertyBranch_t* disjoint_branch = nullptr;
        getInMap(&disjoint_branch, disjoints[disjoints_j], roots_);
        getInMap(&disjoint_branch, disjoints[disjoints_j], branchs_);
        getInMap(&disjoint_branch, disjoints[disjoints_j], tmp_mothers_);

        //I create my disjoint
        if(disjoint_branch == nullptr)
        {
          disjoint_branch = new struct DataPropertyBranch_t(disjoints[disjoints_j]);
          tmp_mothers_[disjoint_branch->value()] = disjoint_branch; //I put my disjoint as tmp_mother
        }
        conditionalPushBack(me->disjoints_, DataPropertyElement_t(disjoint_branch));
        conditionalPushBack(disjoint_branch->disjoints_, DataPropertyElement_t(me, 1.0, true));
      }
    }
  }
}


std::unordered_set<std::string> DataPropertyGraph::getDisjoint(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(auto& disjoint : branch->disjoints_)
      getDown(disjoint.elem, res);

  return res;
}

std::unordered_set<std::string> DataPropertyGraph::getDomain(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(auto& domain : branch->domains_)
      class_graph_->getDown(domain.elem, res);

  return res;
}

void DataPropertyGraph::getDomainPtr(DataPropertyBranch_t* branch, std::unordered_set<ClassBranch_t*>& res, size_t depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  if(branch != nullptr)
    for(auto& domain : branch->domains_)
      class_graph_->getDownPtr(domain.elem, res, depth);
}

std::unordered_set<std::string> DataPropertyGraph::getRange(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned range_i = 0; range_i < branch->ranges_.size(); range_i++)
      res.insert(branch->ranges_[range_i].type_);

  return res;
}

std::unordered_set<std::string> DataPropertyGraph::select(std::unordered_set<std::string>& on, const std::string& selector)
{
  std::unordered_set<std::string> res;
  for(const std::string& it : on)
  {
    std::unordered_set<std::string> tmp = getUp(it);
    if(tmp.find(selector) != tmp.end())
      res.insert(it);
  }
  return res;
}

bool DataPropertyGraph::add(DataPropertyBranch_t* prop, std::string& relation, std::string& data)
{
  if(relation != "")
  {
    if(relation[0] == '@')
    {
      relation = relation.substr(1);
      std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      prop->setSteady_dictionary(relation, data);
      prop->updated_ = true;
    }
    else if((relation == "+") || (relation == "isA"))
    {
      DataPropertyBranch_t* tmp = create(data);
      std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      conditionalPushBack(prop->mothers_, DataPropertyElement_t(tmp));
      conditionalPushBack(tmp->childs_, DataPropertyElement_t(prop));
      prop->updated_ = true;
      tmp->updated_ = true;
    }
    else
      return false;
  }
  else
    return false;
  return true;
}

bool DataPropertyGraph::addInvert(DataPropertyBranch_t* prop, std::string& relation, std::string& data)
{
  if(relation != "")
  {
    if((relation == "+") || (relation == "isA"))
    {
      DataPropertyBranch_t* tmp = create(data);
      std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      conditionalPushBack(tmp->mothers_, DataPropertyElement_t(prop));
      conditionalPushBack(prop->childs_, DataPropertyElement_t(tmp));
      prop->updated_ = true;
      tmp->updated_ = true;
    }
    else
      return false;
  }
  else
    return false;
  return true;
}

bool DataPropertyGraph::remove(DataPropertyBranch_t* prop, std::string& relation, std::string& data)
{
  (void)prop;
  (void)relation;
  (void)data;
  return false;
}

} // namespace ontologenius
