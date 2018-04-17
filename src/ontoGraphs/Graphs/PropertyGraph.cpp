#include "ontoloGenius/ontoGraphs/Graphs/PropertyGraph.h"
#include <iostream>

void PropertyGraph::add(std::string value, PropertyVectors_t& property_vectors)
{
/**********************
** Mothers
**********************/
  PropertyClassBranch_t* me = nullptr;
  //am I a created mother ?
  amIA(&me, tmp_mothers_, value);

  //am I a created branch ?
  amIA(&me, branchs_, value);

  //am I a created root ?
  amIA(&me, roots_, value);

  //am I created ?
  if(me == nullptr)
    me = new PropertyClassBranch_t(value);

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
      i_find_my_mother = isMyMother(me, property_vectors.mothers_[mothers_i], roots_, i_find_my_mother);

      //is a branch my mother ?
      i_find_my_mother = isMyMother(me, property_vectors.mothers_[mothers_i], branchs_, i_find_my_mother);

      //is a tmp mother is mine ?
      i_find_my_mother = isMyMother(me, property_vectors.mothers_[mothers_i], tmp_mothers_, i_find_my_mother);

      //I create my mother
      if(!i_find_my_mother)
      {
        PropertyClassBranch_t* my_mother = new struct PropertyClassBranch_t(property_vectors.mothers_[mothers_i]);
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
    for(unsigned int root_i = 0; root_i < roots_.size(); root_i++)
      if(property_vectors.disjoints_[disjoints_i] == roots_[root_i]->value_)
      {
        me->disjoints_.push_back(roots_[root_i]);
        roots_[root_i]->disjoints_.push_back(me);
        i_find_my_disjoint = true;
      }

    //is a branch my disjoint ?
    for(unsigned int branch_i = 0; branch_i < branchs_.size(); branch_i++)
      if(property_vectors.disjoints_[disjoints_i] == branchs_[branch_i]->value_)
      {
        me->disjoints_.push_back(branchs_[branch_i]);
        branchs_[branch_i]->disjoints_.push_back(me);
        i_find_my_disjoint = true;
      }

    //is a tmp mother is my disjoint ?
    for(unsigned int branch_i  =0; branch_i < tmp_mothers_.size(); branch_i++)
      if(property_vectors.disjoints_[disjoints_i] == tmp_mothers_[branch_i]->value_)
      {
        me->disjoints_.push_back(tmp_mothers_[branch_i]);
        tmp_mothers_[branch_i]->disjoints_.push_back(me);
        i_find_my_disjoint = true;
      }

    //I create my disjoint
    if(!i_find_my_disjoint)
    {
      PropertyClassBranch_t* my_disjoint = new struct PropertyClassBranch_t(property_vectors.disjoints_[disjoints_i]);
      me->disjoints_.push_back(my_disjoint);
      my_disjoint->disjoints_.push_back(me);
      tmp_mothers_.push_back(my_disjoint); //I put my disjoint as tmp_mother
    }
  }

  /**********************
  ** Inverses
  **********************/
  //for all my inverses
  for(unsigned int inverses_i = 0; inverses_i < property_vectors.inverses_.size(); inverses_i++)
  {
    bool i_find_my_inverse = false;

    //is a root my inverse ?
    for(unsigned int root_i = 0; root_i < roots_.size(); root_i++)
      if(property_vectors.inverses_[inverses_i] == roots_[root_i]->value_)
      {
        me->inverses_.push_back(roots_[root_i]);
        roots_[root_i]->inverses_.push_back(me);
        i_find_my_inverse = true;
      }

    //is a branch my inverse ?
    for(unsigned int branch_i = 0; branch_i < branchs_.size(); branch_i++)
      if(property_vectors.inverses_[inverses_i] == branchs_[branch_i]->value_)
      {
        me->inverses_.push_back(branchs_[branch_i]);
        branchs_[branch_i]->inverses_.push_back(me);
        i_find_my_inverse = true;
      }

    //is a tmp mother is my inverse ?
    for(unsigned int branch_i  =0; branch_i < tmp_mothers_.size(); branch_i++)
      if(property_vectors.inverses_[inverses_i] == tmp_mothers_[branch_i]->value_)
      {
        me->inverses_.push_back(tmp_mothers_[branch_i]);
        tmp_mothers_[branch_i]->inverses_.push_back(me);
        i_find_my_inverse = true;
      }

    //I create my inverse
    if(!i_find_my_inverse)
    {
      PropertyClassBranch_t* my_inverse = new struct PropertyClassBranch_t(property_vectors.inverses_[inverses_i]);
      me->inverses_.push_back(my_inverse);
      my_inverse->inverses_.push_back(me);
      tmp_mothers_.push_back(my_inverse); //I put my inverse as tmp_mother
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
    for(unsigned int root_i = 0; root_i < treeObject_->roots_.size(); root_i++)
      if(property_vectors.domains_[domains_i] == treeObject_->roots_[root_i]->value_)
      {
        me->domains_.push_back(treeObject_->roots_[root_i]);
        i_find_my_domain = true;
      }

    //is a branch my domain ?
    for(unsigned int branch_i = 0; branch_i < treeObject_->branchs_.size(); branch_i++)
      if(property_vectors.domains_[domains_i] == treeObject_->branchs_[branch_i]->value_)
      {
        me->domains_.push_back(treeObject_->branchs_[branch_i]);
        i_find_my_domain = true;
      }

    //is a tmp mother is my domain ?
    for(unsigned int branch_i  =0; branch_i < treeObject_->tmp_mothers_.size(); branch_i++)
      if(property_vectors.domains_[domains_i] == treeObject_->tmp_mothers_[branch_i]->value_)
      {
        me->domains_.push_back(treeObject_->tmp_mothers_[branch_i]);
        i_find_my_domain = true;
      }

    //I create my domain
    if(!i_find_my_domain)
    {
      ObjectVectors_t empty_vectors;
      treeObject_->add(property_vectors.domains_[domains_i], empty_vectors);
      for(unsigned int root_i = 0; root_i < treeObject_->roots_.size(); root_i++)
        if(property_vectors.domains_[domains_i] == treeObject_->roots_[root_i]->value_)
        {
          me->domains_.push_back(treeObject_->roots_[root_i]);
          i_find_my_domain = true;
        }
    }
  }

  /**********************
  ** Ranges
  **********************/
  //for all my ranges
  for(unsigned int ranges_i = 0; ranges_i < property_vectors.ranges_.size(); ranges_i++)
  {
    bool i_find_my_range = false;

    //is a root my range ?
    for(unsigned int root_i = 0; root_i < treeObject_->roots_.size(); root_i++)
      if(property_vectors.ranges_[ranges_i] == treeObject_->roots_[root_i]->value_)
      {
        me->ranges_.push_back(treeObject_->roots_[root_i]);
        i_find_my_range = true;
      }

    //is a branch my range ?
    for(unsigned int branch_i = 0; branch_i < treeObject_->branchs_.size(); branch_i++)
      if(property_vectors.ranges_[ranges_i] == treeObject_->branchs_[branch_i]->value_)
      {
        me->ranges_.push_back(treeObject_->branchs_[branch_i]);
        i_find_my_range = true;
      }

    //is a tmp mother is my range ?
    for(unsigned int branch_i  =0; branch_i < treeObject_->tmp_mothers_.size(); branch_i++)
      if(property_vectors.ranges_[ranges_i] == treeObject_->tmp_mothers_[branch_i]->value_)
      {
        me->ranges_.push_back(treeObject_->tmp_mothers_[branch_i]);
        i_find_my_range = true;
      }

    //I create my range
    if(!i_find_my_range)
    {
      ObjectVectors_t empty_vectors;
      treeObject_->add(property_vectors.ranges_[ranges_i], empty_vectors);
      for(unsigned int root_i = 0; root_i < treeObject_->roots_.size(); root_i++)
        if(property_vectors.ranges_[ranges_i] == treeObject_->roots_[root_i]->value_)
        {
          me->ranges_.push_back(treeObject_->roots_[root_i]);
          i_find_my_range = true;
        }
    }
  }

  //////
  // Construction and language properties
  //////
  me->properties_ = property_vectors.properties_;
  me->dictionary_ = property_vectors.dictionary_;
  if(me->dictionary_.find("en") == me->dictionary_.end())
    me->dictionary_["en"] = me->value_;
}

void PropertyGraph::add(std::vector<std::string>& disjoints)
{
  for(unsigned int disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
  {
    //I need to find myself
    PropertyClassBranch_t* me = nullptr;
    //Am I a root ?
    amIA(&me, roots_, disjoints[disjoints_i], false);

    //Am I a branch ?
    amIA(&me, branchs_, disjoints[disjoints_i], false);

    //Am I a tmp_mother ?
    amIA(&me, tmp_mothers_, disjoints[disjoints_i], false);

    // I don't exist ? so I will be a tmp_mother
    if(me == nullptr)
    {
      me = new struct PropertyClassBranch_t(disjoints[disjoints_i]);
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
        for(unsigned int root_i = 0; root_i < roots_.size(); root_i++)
          if(disjoints[disjoints_j] == roots_[root_i]->value_)
          {
            me->disjoints_.push_back(roots_[root_i]);
            i_find_my_disjoint = true;
          }

        //is a branch my disjoint ?
        for(unsigned int branch_i = 0; branch_i < branchs_.size(); branch_i++)
          if(disjoints[disjoints_j] == branchs_[branch_i]->value_)
          {
            me->disjoints_.push_back(branchs_[branch_i]);
            i_find_my_disjoint = true;
          }

        //is a tmp mother is my disjoint ?
        for(unsigned int branch_i  =0; branch_i < tmp_mothers_.size(); branch_i++)
          if(disjoints[disjoints_j] == tmp_mothers_[branch_i]->value_)
          {
            me->disjoints_.push_back(tmp_mothers_[branch_i]);
            i_find_my_disjoint = true;
          }

        //I create my disjoint
        if(!i_find_my_disjoint)
        {
          PropertyClassBranch_t* my_disjoint = new struct PropertyClassBranch_t(disjoints[disjoints_j]);
          me->disjoints_.push_back(my_disjoint);
          tmp_mothers_.push_back(my_disjoint); //I put my disjoint as tmp_mother
        }
      }
    }
  }
}


std::set<std::string> PropertyGraph::getDisjoint(std::string& value)
{
  std::set<std::string> res;

  PropertyClassBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned disjoint_i = 0; disjoint_i < branch->disjoints_.size(); disjoint_i++)
    {
      std::set<std::string> tmp = getDown(branch->disjoints_[disjoint_i]);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  return res;
}

std::set<std::string> PropertyGraph::getInverse(std::string& value)
{
  std::set<std::string> res;

  PropertyClassBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned inverse_i = 0; inverse_i < branch->inverses_.size(); inverse_i++)
    {
      std::set<std::string> tmp = getDown(branch->inverses_[inverse_i]);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  return res;
}

std::set<std::string> PropertyGraph::getDomain(std::string& value)
{
  std::set<std::string> res;

  PropertyClassBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned domain_i = 0; domain_i < branch->domains_.size(); domain_i++)
    {
      std::set<std::string> tmp = treeObject_->getDown(branch->domains_[domain_i]);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  return res;
}

std::set<std::string> PropertyGraph::getRange(std::string& value)
{
  std::set<std::string> res;

  PropertyClassBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(unsigned range_i = 0; range_i < branch->ranges_.size(); range_i++)
    {
      std::set<std::string> tmp = treeObject_->getDown(branch->ranges_[range_i]);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  return res;
}

std::set<std::string> PropertyGraph::select(std::set<std::string> on, std::string selector)
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
