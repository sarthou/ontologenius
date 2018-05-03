#include "ontoloGenius/ontoGraphs/Graphs/DataPropertyGraph.h"
#include <iostream>

void DataPropertyGraph::add(std::string value, DataPropertyVectors_t& property_vectors)
{
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
    roots_.push_back(me);
  else
  {
    //for all my mothers
    for(unsigned int mothers_i = 0; mothers_i < property_vectors.mothers_.size(); mothers_i++)
    {
      bool i_find_my_mother = false;

      //is a root my mother ?
      isMyMother(me, property_vectors.mothers_[mothers_i], roots_, i_find_my_mother);

      //is a branch my mother ?
      isMyMother(me, property_vectors.mothers_[mothers_i], branchs_, i_find_my_mother);

      //is a tmp mother is mine ?
      isMyMother(me, property_vectors.mothers_[mothers_i], tmp_mothers_, i_find_my_mother);

      //I create my mother
      if(!i_find_my_mother)
      {
        DataPropertyBranch_t* my_mother = new struct DataPropertyBranch_t(property_vectors.mothers_[mothers_i]);
        my_mother->childs_.push_back(me);
        me->mothers_.push_back(my_mother);
        tmp_mothers_.push_back(my_mother);
      }
    }

    //but i am also a branch
    branchs_.push_back(me);
  }

  /**********************
  ** Disjoints
  **********************/
  //for all my disjoints
  for(unsigned int disjoints_i = 0; disjoints_i < property_vectors.disjoints_.size(); disjoints_i++)
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
      DataPropertyBranch_t* my_disjoint = new struct DataPropertyBranch_t(property_vectors.disjoints_[disjoints_i]);
      me->disjoints_.push_back(my_disjoint);
      my_disjoint->disjoints_.push_back(me);
      tmp_mothers_.push_back(my_disjoint); //I put my disjoint as tmp_mother
    }
  }

  /**********************
  ** Domains
  **********************/
  //for all my domains
  for(unsigned int domains_i = 0; domains_i < property_vectors.domains_.size(); domains_i++)
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
      for(unsigned int root_i = 0; root_i < class_graph_->roots_.size(); root_i++)
        if(property_vectors.domains_[domains_i] == class_graph_->roots_[root_i]->value_)
        {
          me->domains_.push_back(class_graph_->roots_[root_i]);
          i_find_my_domain = true;
        }
    }
  }

  /**********************
  ** Ranges
  **********************/
  //for all my ranges
  for(unsigned int ranges_i = 0; ranges_i < property_vectors.ranges_.size(); ranges_i++)
    me->ranges_.push_back(property_vectors.ranges_[ranges_i]);

  /**********************
  ** Language and properties
  **********************/
  me->properties_ = property_vectors.properties_;
  me->dictionary_ = property_vectors.dictionary_;
  if(me->dictionary_.find("en") == me->dictionary_.end())
    me->dictionary_["en"] = me->value_;
}

void DataPropertyGraph::add(std::vector<std::string>& disjoints)
{
  for(unsigned int disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
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
      tmp_mothers_.push_back(me);
    }

    //for all my disjoints ...
    for(unsigned int disjoints_j = 0; disjoints_j < disjoints.size(); disjoints_j++)
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
          DataPropertyBranch_t* my_disjoint = new struct DataPropertyBranch_t(disjoints[disjoints_j]);
          me->disjoints_.push_back(my_disjoint);
          tmp_mothers_.push_back(my_disjoint); //I put my disjoint as tmp_mother
        }
      }
    }
  }
}


std::set<std::string> DataPropertyGraph::getDisjoint(std::string& value)
{
  std::set<std::string> res;

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned disjoint_i = 0; disjoint_i < branch->disjoints_.size(); disjoint_i++)
    {
      std::set<std::string> tmp = getDown(branch->disjoints_[disjoint_i]);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  return res;
}

std::set<std::string> DataPropertyGraph::getDomain(std::string& value)
{
  std::set<std::string> res;

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned domain_i = 0; domain_i < branch->domains_.size(); domain_i++)
    {
      std::set<std::string> tmp = class_graph_->getDown(branch->domains_[domain_i]);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  return res;
}

std::set<std::string> DataPropertyGraph::getRange(std::string& value)
{
  std::set<std::string> res;

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned range_i = 0; range_i < branch->ranges_.size(); range_i++)
      res.insert(branch->ranges_[range_i]);

  return res;
}

std::set<std::string> DataPropertyGraph::select(std::set<std::string> on, std::string selector)
{
  std::set<std::string> res;
  for(std::set<std::string>::iterator it = on.begin(); it != on.end(); ++it)
  {
    std::string prop_i = *it;
    std::set<std::string> tmp = getUp(prop_i);
    if(tmp.find(selector) != tmp.end())
      res.insert(*it);
  }
  return res;
}
