#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

#include <iostream>

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"

namespace ontologenius {

void ObjectPropertyGraph::add(std::string value, ObjectPropertyVectors_t& property_vectors)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);
  /**********************
  ** Mothers
  **********************/
  ObjectPropertyBranch_t* me = nullptr;
  //am I a created mother ?
  amIA(&me, tmp_mothers_, value);

  //am I a created branch ?
  amIA(&me, branchs_, value);

  //am I a created root ?
  amIA(&me, roots_, value);

  //am I created ?
  if(me == nullptr)
    me = new ObjectPropertyBranch_t(value);

  me->nb_mothers_ += property_vectors.mothers_.size();

  //am I a root ?
  if(me->nb_mothers_ == 0)
    roots_[value] = me;
  else
  {
    //for all my mothers
    for(size_t mothers_i = 0; mothers_i < property_vectors.mothers_.size(); mothers_i++)
    {
      bool i_find_my_mother = false;

      //is a root my mother ?
      isMyMother(me, property_vectors.mothers_[mothers_i].elem, roots_, i_find_my_mother);

      //is a branch my mother ?
      isMyMother(me, property_vectors.mothers_[mothers_i].elem, branchs_, i_find_my_mother);

      //is a tmp mother is mine ?
      isMyMother(me, property_vectors.mothers_[mothers_i].elem, tmp_mothers_, i_find_my_mother);

      //I create my mother
      if(!i_find_my_mother)
      {
        ObjectPropertyBranch_t* my_mother = new struct ObjectPropertyBranch_t(property_vectors.mothers_[mothers_i].elem);
        my_mother->childs_.push_back(ObjectPropertyElement_t(me));
        me->setSteady_mother(ObjectPropertyElement_t(my_mother));
        tmp_mothers_[my_mother->value()] = my_mother;
      }
    }

    //but i am also a branch
    branchs_[me->value()] = me;
  }

  /**********************
  ** Disjoints
  **********************/
  //for all my disjoints
  for(size_t disjoints_i = 0; disjoints_i < property_vectors.disjoints_.size(); disjoints_i++)
  {
    bool i_find_my_disjoint = false;

    //is a root my disjoint ?
    isMyDisjoint(me, property_vectors.disjoints_[disjoints_i], roots_, i_find_my_disjoint);

    //is a branch my disjoint ?
    isMyDisjoint(me, property_vectors.disjoints_[disjoints_i], branchs_, i_find_my_disjoint);

    //is a tmp mother is my disjoint ?
    isMyDisjoint(me, property_vectors.disjoints_[disjoints_i], tmp_mothers_, i_find_my_disjoint);

    //I create my disjoint
    if(!i_find_my_disjoint)
    {
      ObjectPropertyBranch_t* my_disjoint = new struct ObjectPropertyBranch_t(property_vectors.disjoints_[disjoints_i]);
      me->setSteady_disjoint(my_disjoint);
      my_disjoint->disjoints_.push_back(me);
      tmp_mothers_[my_disjoint->value()] = my_disjoint; //I put my disjoint as tmp_mother
    }
  }

  /**********************
  ** Inverses
  **********************/
  //for all my inverses
  for(size_t inverses_i = 0; inverses_i < property_vectors.inverses_.size(); inverses_i++)
  {
    bool i_find_my_inverse = false;

    //is a root my inverse ?
    isMyInverse(me, property_vectors.inverses_[inverses_i], roots_, i_find_my_inverse);

    //is a branch my inverse ?
    isMyInverse(me, property_vectors.inverses_[inverses_i], branchs_, i_find_my_inverse);

    //is a tmp mother is my inverse ?
    isMyInverse(me, property_vectors.inverses_[inverses_i], tmp_mothers_, i_find_my_inverse);

    //I create my inverse
    if(!i_find_my_inverse)
    {
      ObjectPropertyBranch_t* my_inverse = new struct ObjectPropertyBranch_t(property_vectors.inverses_[inverses_i]);
      me->setSteady_inverse(my_inverse);
      my_inverse->inverses_.push_back(me);
      tmp_mothers_[my_inverse->value()] = my_inverse; //I put my inverse as tmp_mother
    }
  }

  /**********************
  ** Domains
  **********************/
  //for all my domains
  for(size_t domains_i = 0; domains_i < property_vectors.domains_.size(); domains_i++)
  {
    bool i_find_my_domain = false;

    //is a root my domain ?
    isMyDomain(me, property_vectors.domains_[domains_i], class_graph_->roots_, i_find_my_domain);

    //is a branch my domain ?
    isMyDomain(me, property_vectors.domains_[domains_i], class_graph_->branchs_, i_find_my_domain);

    //is a tmp mother is my domain ?
    isMyDomain(me, property_vectors.domains_[domains_i], class_graph_->tmp_mothers_, i_find_my_domain);

    //I create my domain
    if(!i_find_my_domain)
    {
      ObjectVectors_t empty_vectors;
      class_graph_->add(property_vectors.domains_[domains_i], empty_vectors);
      auto it = class_graph_->roots_.find(property_vectors.domains_[domains_i]);
      if(it != class_graph_->roots_.end())
      {
        me->setSteady_domain(it->second);
        i_find_my_domain = true;
      }
    }
  }

  /**********************
  ** Ranges
  **********************/
  //for all my ranges
  for(size_t ranges_i = 0; ranges_i < property_vectors.ranges_.size(); ranges_i++)
  {
    bool i_find_my_range = false;

    //is a root my range ?
    isMyRange(me, property_vectors.ranges_[ranges_i], class_graph_->roots_, i_find_my_range);

    //is a branch my range ?
    isMyRange(me, property_vectors.ranges_[ranges_i], class_graph_->branchs_, i_find_my_range);

    //is a tmp mother is my range ?
    isMyRange(me, property_vectors.ranges_[ranges_i], class_graph_->tmp_mothers_, i_find_my_range);

    //I create my range
    if(!i_find_my_range)
    {
      ObjectVectors_t empty_vectors;
      class_graph_->add(property_vectors.ranges_[ranges_i], empty_vectors);
      auto it = class_graph_->roots_.find(property_vectors.ranges_[ranges_i]);
      if(it != class_graph_->roots_.end())
      {
        me->setSteady_range(it->second);
        i_find_my_range = true;
      }
    }
  }

  /**********************
  ** Language and properties
  **********************/
  me->setSteady_properties(property_vectors.properties_);
  me->setSteady_dictionary(property_vectors.dictionary_);
  if(me->dictionary_.find("en") == me->dictionary_.end())
    me->dictionary_["en"].push_back(me->value());
  me->setSteady_muted_dictionary(property_vectors.muted_dictionary_);

  /**********************
  ** Chain axiom
  **********************/
  for(size_t chain_i = 0; chain_i < property_vectors.chains_.size(); chain_i++)
  {
    std::vector<ObjectPropertyBranch_t*> chain;
    ObjectPropertyBranch_t* first = nullptr;

    for(size_t i = 0; i < property_vectors.chains_[chain_i].size(); i++)
    {
      ObjectPropertyBranch_t* next = nullptr;

      //is a root my next ?
      getNextChainLink(&next, property_vectors.chains_[chain_i][i], roots_);

      //is a branch my next ?
      getNextChainLink(&next, property_vectors.chains_[chain_i][i], branchs_);

      //is a tmp mother is my next ?
      getNextChainLink(&next, property_vectors.chains_[chain_i][i], tmp_mothers_);

      if(next == nullptr)
      {
        next = new struct ObjectPropertyBranch_t(property_vectors.chains_[chain_i][i]);
        tmp_mothers_[next->value()] = next;
      }

      if(first == nullptr)
        first = next;
      else
        chain.push_back(next);
    }

    chain.push_back(me);
    first->set_chain(chain);
    me->setSteady_chain(property_vectors.chains_[chain_i]);
  }

  mitigate(me);
}

void ObjectPropertyGraph::add(std::vector<std::string>& disjoints)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);

  for(size_t disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
  {
    //I need to find myself
    ObjectPropertyBranch_t* me = nullptr;
    //Am I a root ?
    amIA(&me, roots_, disjoints[disjoints_i], false);

    //Am I a branch ?
    amIA(&me, branchs_, disjoints[disjoints_i], false);

    //Am I a tmp_mother ?
    amIA(&me, tmp_mothers_, disjoints[disjoints_i], false);

    // I don't exist ? so I will be a tmp_mother
    if(me == nullptr)
    {
      me = new struct ObjectPropertyBranch_t(disjoints[disjoints_i]);
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
          ObjectPropertyBranch_t* my_disjoint = new struct ObjectPropertyBranch_t(disjoints[disjoints_j]);
          me->setSteady_disjoint(my_disjoint);
          tmp_mothers_[my_disjoint->value()] = my_disjoint; //I put my disjoint as tmp_mother
        }
      }
    }
  }
}


std::unordered_set<std::string> ObjectPropertyGraph::getDisjoint(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);

  ObjectPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned disjoint_i = 0; disjoint_i < branch->disjoints_.size(); disjoint_i++)
      getDown(branch->disjoints_[disjoint_i], res);

  return res;
}

void ObjectPropertyGraph::getDisjoint(ObjectPropertyBranch_t* branch, std::unordered_set<ObjectPropertyBranch_t*>& res)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);

  if(branch != nullptr)
    for(unsigned disjoint_i = 0; disjoint_i < branch->disjoints_.size(); disjoint_i++)
      getDownPtr(branch->disjoints_[disjoint_i], res);
}

std::unordered_set<std::string> ObjectPropertyGraph::getInverse(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);

  ObjectPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned inverse_i = 0; inverse_i < branch->inverses_.size(); inverse_i++)
      getDown(branch->inverses_[inverse_i], res);

  return res;
}

std::unordered_set<std::string> ObjectPropertyGraph::getDomain(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);

  ObjectPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned domain_i = 0; domain_i < branch->domains_.size(); domain_i++)
      class_graph_->getDown(branch->domains_[domain_i], res);

  return res;
}

void ObjectPropertyGraph::getDomainPtr(ObjectPropertyBranch_t* branch, std::unordered_set<ClassBranch_t*>& res, size_t depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);

  if(branch != nullptr)
    for(unsigned domain_i = 0; domain_i < branch->domains_.size(); domain_i++)
      class_graph_->getDownPtr(branch->domains_[domain_i], res, depth);
}

std::unordered_set<std::string> ObjectPropertyGraph::getRange(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);

  ObjectPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned range_i = 0; range_i < branch->ranges_.size(); range_i++)
      class_graph_->getDown(branch->ranges_[range_i], res);

  return res;
}

void ObjectPropertyGraph::getRangePtr(ObjectPropertyBranch_t* branch, std::unordered_set<ClassBranch_t*>& res, size_t depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<ObjectPropertyBranch_t>::mutex_);
  if(branch != nullptr)
    for(unsigned range_i = 0; range_i < branch->ranges_.size(); range_i++)
      class_graph_->getDownPtr(branch->ranges_[range_i], res, depth);
}

std::unordered_set<std::string> ObjectPropertyGraph::select(std::unordered_set<std::string>& on, const std::string& selector)
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

bool ObjectPropertyGraph::add(ObjectPropertyBranch_t* prop, std::string& relation, std::string& data)
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
      ObjectPropertyBranch_t* tmp = create(data);
      std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      prop->setSteady_mother(tmp);
      tmp->setSteady_child(prop);
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

bool ObjectPropertyGraph::addInvert(ObjectPropertyBranch_t* prop, std::string& relation, std::string& data)
{
  if(relation != "")
  {
    if((relation == "+") || (relation == "isA"))
    {
      ObjectPropertyBranch_t* tmp = create(data);
      std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      tmp->setSteady_mother(prop);
      prop->setSteady_child(tmp);
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

bool ObjectPropertyGraph::remove(ObjectPropertyBranch_t* prop, std::string& relation, std::string& data)
{
  (void)prop;
  (void)relation;
  (void)data;
  return false;
}

} // namespace ontologenius
