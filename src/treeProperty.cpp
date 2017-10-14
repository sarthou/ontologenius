#include "ontoloGenius/treeProperty.h"
#include <iostream>

using namespace std;

treeProperty::~treeProperty()
{
  for(unsigned int i = 0; i < branchs.size(); i++)
    delete branchs[i];

  for(unsigned int i = 0; i < roots.size(); i++)
    delete roots[i];
}

void treeProperty::add(string value, PropertyVectors_t& propertyVectors)
{
/**********************
** Mothers
**********************/
  //am I a created mother ?
  propertyBranch_t* me = nullptr;
  for(unsigned int i = 0; i < tmp_mothers.size(); i++)
  {
    if(tmp_mothers[i]->value == value)
    {
      me = tmp_mothers[i];
      tmp_mothers.erase(tmp_mothers.begin() + i);
      break;
    }
  }

  //am I created ?
  if(me == nullptr)
    me = new struct propertyBranch_t(value);

  me->nb_mothers = propertyVectors.mothers.size();

  //am I a root ?
  if(me->nb_mothers == 0)
    roots.push_back(me);
  else
  {
    //for all my mothers
    for(unsigned int mothers_i = 0; mothers_i < propertyVectors.mothers.size(); mothers_i++)
    {
      bool i_find_my_mother = false;

      //is a root my mother ?
      for(unsigned int root_i = 0; root_i < roots.size(); root_i++)
        if(propertyVectors.mothers[mothers_i] == roots[root_i]->value)
        {
          roots[root_i]->childs.push_back(me);
          me->mothers.push_back(roots[root_i]);
          i_find_my_mother = true;
        }

      //is a branch my mother ?
      for(unsigned int branch_i = 0; branch_i < branchs.size(); branch_i++)
        if(propertyVectors.mothers[mothers_i] == branchs[branch_i]->value)
        {
          branchs[branch_i]->childs.push_back(me);
          me->mothers.push_back(branchs[branch_i]);
          i_find_my_mother = true;
        }

      //is a tmp mother is mine ?
      for(unsigned int branch_i  =0; branch_i < tmp_mothers.size(); branch_i++)
        if(propertyVectors.mothers[mothers_i] == tmp_mothers[branch_i]->value)
        {
          tmp_mothers[branch_i]->childs.push_back(me);
          me->mothers.push_back(tmp_mothers[branch_i]);
          i_find_my_mother = true;
        }

      //I create my mother
      if(!i_find_my_mother)
      {
        propertyBranch_t* my_mother = new struct propertyBranch_t(propertyVectors.mothers[mothers_i]);
        my_mother->childs.push_back(me);
        me->mothers.push_back(my_mother);
        tmp_mothers.push_back(my_mother);
      }
    }

    //but i am also a branch
    branchs.push_back(me);
  }

  /**********************
  ** Disjoints
  **********************/
  //for all my disjoints
  for(unsigned int disjoints_i = 0; disjoints_i < propertyVectors.disjoints.size(); disjoints_i++)
  {
    bool i_find_my_disjoint = false;

    //is a root my disjoint ?
    for(unsigned int root_i = 0; root_i < roots.size(); root_i++)
      if(propertyVectors.disjoints[disjoints_i] == roots[root_i]->value)
      {
        me->disjoints.push_back(roots[root_i]);
        roots[root_i]->disjoints.push_back(me);
        i_find_my_disjoint = true;
      }

    //is a branch my disjoint ?
    for(unsigned int branch_i = 0; branch_i < branchs.size(); branch_i++)
      if(propertyVectors.disjoints[disjoints_i] == branchs[branch_i]->value)
      {
        me->disjoints.push_back(branchs[branch_i]);
        branchs[branch_i]->disjoints.push_back(me);
        i_find_my_disjoint = true;
      }

    //is a tmp mother is my disjoint ?
    for(unsigned int branch_i  =0; branch_i < tmp_mothers.size(); branch_i++)
      if(propertyVectors.disjoints[disjoints_i] == tmp_mothers[branch_i]->value)
      {
        me->disjoints.push_back(tmp_mothers[branch_i]);
        tmp_mothers[branch_i]->disjoints.push_back(me);
        i_find_my_disjoint = true;
      }

    //I create my disjoint
    if(!i_find_my_disjoint)
    {
      propertyBranch_t* my_disjoint = new struct propertyBranch_t(propertyVectors.disjoints[disjoints_i]);
      me->disjoints.push_back(my_disjoint);
      my_disjoint->disjoints.push_back(me);
      tmp_mothers.push_back(my_disjoint); //I put my disjoint as tmp_mother
    }
  }

  /**********************
  ** Inverses
  **********************/
  //for all my inverses
  for(unsigned int inverses_i = 0; inverses_i < propertyVectors.inverses.size(); inverses_i++)
  {
    bool i_find_my_inverse = false;

    //is a root my inverse ?
    for(unsigned int root_i = 0; root_i < roots.size(); root_i++)
      if(propertyVectors.inverses[inverses_i] == roots[root_i]->value)
      {
        me->inverses.push_back(roots[root_i]);
        roots[root_i]->inverses.push_back(me);
        i_find_my_inverse = true;
      }

    //is a branch my inverse ?
    for(unsigned int branch_i = 0; branch_i < branchs.size(); branch_i++)
      if(propertyVectors.inverses[inverses_i] == branchs[branch_i]->value)
      {
        me->inverses.push_back(branchs[branch_i]);
        branchs[branch_i]->inverses.push_back(me);
        i_find_my_inverse = true;
      }

    //is a tmp mother is my inverse ?
    for(unsigned int branch_i  =0; branch_i < tmp_mothers.size(); branch_i++)
      if(propertyVectors.inverses[inverses_i] == tmp_mothers[branch_i]->value)
      {
        me->inverses.push_back(tmp_mothers[branch_i]);
        tmp_mothers[branch_i]->inverses.push_back(me);
        i_find_my_inverse = true;
      }

    //I create my inverse
    if(!i_find_my_inverse)
    {
      propertyBranch_t* my_inverse = new struct propertyBranch_t(propertyVectors.inverses[inverses_i]);
      me->inverses.push_back(my_inverse);
      my_inverse->inverses.push_back(me);
      tmp_mothers.push_back(my_inverse); //I put my inverse as tmp_mother
    }
  }

  /**********************
  ** Domains
  **********************/
  //for all my domains
  for(unsigned int domains_i = 0; domains_i < propertyVectors.domains.size(); domains_i++)
  {
    bool i_find_my_domain = false;

    //is a root my domain ?
    for(unsigned int root_i = 0; root_i < treeObject_->roots.size(); root_i++)
      if(propertyVectors.domains[domains_i] == treeObject_->roots[root_i]->value)
      {
        me->domains.push_back(treeObject_->roots[root_i]);
        i_find_my_domain = true;
      }

    //is a branch my domain ?
    for(unsigned int branch_i = 0; branch_i < treeObject_->branchs.size(); branch_i++)
      if(propertyVectors.domains[domains_i] == treeObject_->branchs[branch_i]->value)
      {
        me->domains.push_back(treeObject_->branchs[branch_i]);
        i_find_my_domain = true;
      }

    //is a tmp mother is my domain ?
    for(unsigned int branch_i  =0; branch_i < treeObject_->tmp_mothers.size(); branch_i++)
      if(propertyVectors.domains[domains_i] == treeObject_->tmp_mothers[branch_i]->value)
      {
        me->domains.push_back(treeObject_->tmp_mothers[branch_i]);
        i_find_my_domain = true;
      }

    //I create my domain
    if(!i_find_my_domain)
    {
      vector<string> empty_mothers, empty_disjoints;
      treeObject_->add(propertyVectors.domains[domains_i], empty_mothers, empty_disjoints);
      for(unsigned int root_i = 0; root_i < treeObject_->roots.size(); root_i++)
        if(propertyVectors.domains[domains_i] == treeObject_->roots[root_i]->value)
        {
          me->domains.push_back(treeObject_->roots[root_i]);
          i_find_my_domain = true;
        }
    }
  }

  /**********************
  ** Ranges
  **********************/
  //for all my ranges
  for(unsigned int ranges_i = 0; ranges_i < propertyVectors.ranges.size(); ranges_i++)
  {
    bool i_find_my_range = false;

    //is a root my range ?
    for(unsigned int root_i = 0; root_i < treeObject_->roots.size(); root_i++)
      if(propertyVectors.ranges[ranges_i] == treeObject_->roots[root_i]->value)
      {
        me->ranges.push_back(treeObject_->roots[root_i]);
        i_find_my_range = true;
      }

    //is a branch my range ?
    for(unsigned int branch_i = 0; branch_i < treeObject_->branchs.size(); branch_i++)
      if(propertyVectors.ranges[ranges_i] == treeObject_->branchs[branch_i]->value)
      {
        me->ranges.push_back(treeObject_->branchs[branch_i]);
        i_find_my_range = true;
      }

    //is a tmp mother is my range ?
    for(unsigned int branch_i  =0; branch_i < treeObject_->tmp_mothers.size(); branch_i++)
      if(propertyVectors.ranges[ranges_i] == treeObject_->tmp_mothers[branch_i]->value)
      {
        me->ranges.push_back(treeObject_->tmp_mothers[branch_i]);
        i_find_my_range = true;
      }

    //I create my range
    if(!i_find_my_range)
    {
      vector<string> empty_mothers, empty_disjoints;
      treeObject_->add(propertyVectors.ranges[ranges_i], empty_mothers, empty_disjoints);
      for(unsigned int root_i = 0; root_i < treeObject_->roots.size(); root_i++)
        if(propertyVectors.ranges[ranges_i] == treeObject_->roots[root_i]->value)
        {
          me->ranges.push_back(treeObject_->roots[root_i]);
          i_find_my_range = true;
        }
    }
  }
}

void treeProperty::add(vector<string>& disjoints)
{
  for(unsigned int disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
  {
    //I need to find myself
    propertyBranch_t* me = nullptr;
    //Am I a root ?
    for(unsigned int root_i = 0; root_i < roots.size(); root_i++)
      if(disjoints[disjoints_i] == roots[root_i]->value)
        me = roots[root_i];

    //Am I a branch ?
    if(me == nullptr)
      for(unsigned int branch_i = 0; branch_i < branchs.size(); branch_i++)
        if(disjoints[disjoints_i] == branchs[branch_i]->value)
          me = branchs[branch_i];

    //Am I a tmp_mother ?
    if(me == nullptr)
      for(unsigned int branch_i  =0; branch_i < tmp_mothers.size(); branch_i++)
        if(disjoints[disjoints_i] == tmp_mothers[branch_i]->value)
          me = tmp_mothers[branch_i];

    // I don't exist ? so I will be a tmp_mother
    if(me == nullptr)
    {
      me = new struct propertyBranch_t(disjoints[disjoints_i]);
      tmp_mothers.push_back(me);
    }

    //for all my disjoints ...
    for(unsigned int disjoints_j = 0; disjoints_j < disjoints.size(); disjoints_j++)
    {
      //... excepted me
      if(disjoints_i != disjoints_j)
      {
        bool i_find_my_disjoint = false;

        //is a root my disjoint ?
        for(unsigned int root_i = 0; root_i < roots.size(); root_i++)
          if(disjoints[disjoints_j] == roots[root_i]->value)
          {
            me->disjoints.push_back(roots[root_i]);
            i_find_my_disjoint = true;
          }

        //is a branch my disjoint ?
        for(unsigned int branch_i = 0; branch_i < branchs.size(); branch_i++)
          if(disjoints[disjoints_j] == branchs[branch_i]->value)
          {
            me->disjoints.push_back(branchs[branch_i]);
            i_find_my_disjoint = true;
          }

        //is a tmp mother is my disjoint ?
        for(unsigned int branch_i  =0; branch_i < tmp_mothers.size(); branch_i++)
          if(disjoints[disjoints_j] == tmp_mothers[branch_i]->value)
          {
            me->disjoints.push_back(tmp_mothers[branch_i]);
            i_find_my_disjoint = true;
          }

        //I create my disjoint
        if(!i_find_my_disjoint)
        {
          propertyBranch_t* my_disjoint = new struct propertyBranch_t(disjoints[disjoints_j]);
          me->disjoints.push_back(my_disjoint);
          tmp_mothers.push_back(my_disjoint); //I put my disjoint as tmp_mother
        }
      }
    }
  }
}

void treeProperty::close()
{
  for(unsigned int i = 0; i < tmp_mothers.size(); i++)
    roots.push_back(tmp_mothers[i]);

  tmp_mothers.clear();

  link();
}

set<string> treeProperty::getDown(string value)
{
  set<string> res;

  for(unsigned int i = 0; i < roots.size(); i++)
    if(roots[i]->value == value)
    {
      set<string> tmp = getDown(roots[i], value);

      if(tmp.size())
      {
        res.insert(tmp.begin(), tmp.end());
        break;
      }
    }

  if(!res.size())
    for(unsigned int i = 0; i < branchs.size(); i++)
      if(branchs[i]->value == value)
      {
        set<string> tmp = getDown(branchs[i], value);

        if(tmp.size())
        {
          res.insert(tmp.begin(), tmp.end());
          break;
        }
      }

  return res;
}

set<string> treeProperty::getUp(string value)
{
  set<string> res;

  for(unsigned int i = 0; i < roots.size(); i++)
    if(roots[i]->value == value)
    {
      set<string> tmp = getUp(roots[i], value);

      if(tmp.size())
      {
        res.insert(tmp.begin(), tmp.end());
        break;
      }
    }

  if(!res.size())
    for(unsigned int i = 0; i < branchs.size(); i++)
      if(branchs[i]->value == value)
      {
        set<string> tmp = getUp(branchs[i], value);

        if(tmp.size())
        {
          res.insert(tmp.begin(), tmp.end());
          break;
        }
      }

  return res;
}

set<string> treeProperty::getDisjoint(string value)
{
  set<string> res;

  for(unsigned int i = 0; i < roots.size(); i++)
    if(roots[i]->value == value)
    {
      cout << roots[i]->value << endl;
      for(unsigned disjoint_i = 0; disjoint_i < roots[i]->disjoints.size(); disjoint_i++)
      {
        cout << "------" << roots[i]->disjoints[disjoint_i]->value << endl;
        set<string> tmp = getDown(roots[i]->disjoints[disjoint_i], value);

        if(tmp.size())
        {
          res.insert(tmp.begin(), tmp.end());
          break;
        }
      }
    }

  if(!res.size())
    for(unsigned int i = 0; i < branchs.size(); i++)
      if(branchs[i]->value == value)
      {
        cout << branchs[i]->value << endl;
        for(unsigned disjoint_i = 0; disjoint_i < branchs[i]->disjoints.size(); disjoint_i++)
        {
          cout << "------" << branchs[i]->disjoints[disjoint_i]->value << endl;
          set<string> tmp = getDown(branchs[i]->disjoints[disjoint_i], value);

          if(tmp.size())
            res.insert(tmp.begin(), tmp.end());
        }
        break;
      }

  return res;
}

set<string> treeProperty::getInverse(string value)
{
  set<string> res;

  for(unsigned int i = 0; i < roots.size(); i++)
    if(roots[i]->value == value)
    {
      cout << roots[i]->value << endl;
      for(unsigned inverse_i = 0; inverse_i < roots[i]->inverses.size(); inverse_i++)
      {
        cout << "------" << roots[i]->inverses[inverse_i]->value << endl;
        set<string> tmp = getDown(roots[i]->inverses[inverse_i], value);

        if(tmp.size())
        {
          res.insert(tmp.begin(), tmp.end());
          break;
        }
      }
    }

  if(!res.size())
    for(unsigned int i = 0; i < branchs.size(); i++)
      if(branchs[i]->value == value)
      {
        cout << branchs[i]->value << endl;
        for(unsigned inverse_i = 0; inverse_i < branchs[i]->inverses.size(); inverse_i++)
        {
          cout << "------" << branchs[i]->inverses[inverse_i]->value << endl;
          set<string> tmp = getDown(branchs[i]->inverses[inverse_i], value);

          if(tmp.size())
          {
            res.insert(tmp.begin(), tmp.end());
            break;
          }
        }
      }

  return res;
}

set<string> treeProperty::getDomain(string value)
{
  set<string> res;

  for(unsigned int i = 0; i < roots.size(); i++)
    if(roots[i]->value == value)
    {
      cout << roots[i]->value << endl;
      for(unsigned domain_i = 0; domain_i < roots[i]->domains.size(); domain_i++)
      {
        cout << "------" << roots[i]->domains[domain_i]->value << endl;
        set<string> tmp = treeObject_->getDown(roots[i]->domains[domain_i], value);

        if(tmp.size())
        {
          res.insert(tmp.begin(), tmp.end());
          break;
        }
      }
    }

  if(!res.size())
    for(unsigned int i = 0; i < branchs.size(); i++)
      if(branchs[i]->value == value)
      {
        cout << branchs[i]->value << endl;
        for(unsigned domain_i = 0; domain_i < branchs[i]->domains.size(); domain_i++)
        {
          cout << "------" << branchs[i]->domains[domain_i]->value << endl;
          set<string> tmp = treeObject_->getDown(branchs[i]->domains[domain_i], value);

          if(tmp.size())
          {
            res.insert(tmp.begin(), tmp.end());
            break;
          }
        }
      }

  return res;
}

set<string> treeProperty::getRange(string value)
{
  set<string> res;

  for(unsigned int i = 0; i < roots.size(); i++)
    if(roots[i]->value == value)
    {
      cout << roots[i]->value << "::" << roots[i]->ranges.size() << endl;
      for(unsigned range_i = 0; range_i < roots[i]->ranges.size(); range_i++)
      {
        cout << "------" << roots[i]->ranges[range_i]->value << endl;
        set<string> tmp = treeObject_->getDown(roots[i]->ranges[range_i], value);

        if(tmp.size())
        {
          res.insert(tmp.begin(), tmp.end());
          break;
        }
      }
    }

  if(!res.size())
    for(unsigned int i = 0; i < branchs.size(); i++)
      if(branchs[i]->value == value)
      {
        cout << branchs[i]->value << "::" << branchs[i]->ranges.size() << endl;
        for(unsigned range_i = 0; range_i < branchs[i]->ranges.size(); range_i++)
        {
          cout << "------" << branchs[i]->ranges[range_i]->value << endl;
          set<string> tmp = treeObject_->getDown(branchs[i]->ranges[range_i], value);

          if(tmp.size())
          {
            res.insert(tmp.begin(), tmp.end());
            break;
          }
        }
      }

  return res;
}

void treeProperty::link()
{
  depth = 0;

  uint8_t nb_root_family = roots.size();
  for(uint8_t root_i = 0; root_i < roots.size(); root_i++)
  {
    roots[root_i]->family = 256/(nb_root_family+1) * root_i;
    for(unsigned int i = 0; i < roots[root_i]->childs.size(); i++)
      add_family(roots[root_i]->childs[i], roots[root_i]->family);
  }
}

void treeProperty::add_family(propertyBranch_t* branch, uint8_t family)
{
  branch->family += family/branch->nb_mothers;
  for(unsigned int i = 0; i < branch->childs.size(); i++)
  {
    depth++;
    if(depth < 20)
      add_family(branch->childs[i], family/branch->nb_mothers);
    depth--;
  }
}

set<string> treeProperty::getDown(propertyBranch_t* branch, string value)
{
  set<string> res;
  res.insert(branch->value);
  for(unsigned int i = 0; i < branch->childs.size(); i++)
  {
    set<string> tmp = getDown(branch->childs[i], value);

    if(tmp.size())
      res.insert(tmp.begin(), tmp.end());
  }

  return res;
}

set<string> treeProperty::getUp(propertyBranch_t* branch, string value)
{
  set<string> res;
  res.insert(branch->value);
  for(unsigned int i = 0; i < branch->mothers.size(); i++)
  {
    set<string> tmp = getUp(branch->mothers[i], value);

    if(tmp.size())
      res.insert(tmp.begin(), tmp.end());
  }

  return res;
}
