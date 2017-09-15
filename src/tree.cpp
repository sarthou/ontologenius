#include "ontoloGenius/tree.h"
#include <iostream>

using namespace std;

tree::~tree()
{
  for(unsigned int i = 0; i < branchs.size(); i++)
    delete branchs[i];

  for(unsigned int i = 0; i < roots.size(); i++)
    delete roots[i];
}

void tree::add(string value, vector<string> mothers)
{
  //am I a created mother ?
  branch_t* me = nullptr;
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
    me = new struct branch_t(value);

  me->nb_mothers = mothers.size();

  //am I a root ?
  if(me->nb_mothers == 0)
    roots.push_back(me);
  else
  {
    //for all my mothers
    for(unsigned int mothers_i = 0; mothers_i < mothers.size(); mothers_i++)
    {
      bool i_find_my_mother = false;

      //is a root my mother ?
      for(unsigned int root_i = 0; root_i < roots.size(); root_i++)
        if(mothers[mothers_i] == roots[root_i]->value)
        {
          roots[root_i]->childs.push_back(me);
          me->mothers.push_back(roots[root_i]);
          i_find_my_mother = true;
        }

      //is a branch my mother ?
      for(unsigned int branch_i = 0; branch_i < branchs.size(); branch_i++)
        if(mothers[mothers_i] == branchs[branch_i]->value)
        {
          branchs[branch_i]->childs.push_back(me);
          me->mothers.push_back(branchs[branch_i]);
          i_find_my_mother = true;
        }

      //is a tmp mother is mine ?
      for(unsigned int branch_i  =0; branch_i < tmp_mothers.size(); branch_i++)
        if(mothers[mothers_i] == tmp_mothers[branch_i]->value)
        {
          tmp_mothers[branch_i]->childs.push_back(me);
          me->mothers.push_back(tmp_mothers[branch_i]);
          i_find_my_mother = true;
        }

      //I create my mother
      if(!i_find_my_mother)
      {
        branch_t* my_mother = new struct branch_t(mothers[mothers_i]);
        my_mother->childs.push_back(me);
        me->mothers.push_back(my_mother);
        tmp_mothers.push_back(my_mother);
      }
    }

    //but i am also a branch
    branchs.push_back(me);
  }
}

void tree::close()
{
  for(unsigned int i = 0; i < tmp_mothers.size(); i++)
    roots.push_back(tmp_mothers[i]);

  tmp_mothers.clear();

  link();
}

set<string> tree::getDown(string value)
{
  set<string> res;

  for(unsigned int i = 0; i < roots.size(); i++)
    if(roots[i]->value == value)
    {
      set<string> tmp = getDown(roots[i], value);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  for(unsigned int i = 0; i < branchs.size(); i++)
    if(branchs[i]->value == value)
    {
      set<string> tmp = getDown(branchs[i], value);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  return res;
}

set<string> tree::getUp(string value)
{
  set<string> res;

  for(unsigned int i = 0; i < roots.size(); i++)
    if(roots[i]->value == value)
    {
      set<string> tmp = getUp(roots[i], value);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  for(unsigned int i = 0; i < branchs.size(); i++)
    if(branchs[i]->value == value)
    {
      set<string> tmp = getUp(branchs[i], value);

      if(tmp.size())
        res.insert(tmp.begin(), tmp.end());
    }

  return res;
}

void tree::link()
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

void tree::add_family(branch_t* branch, uint8_t family)
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

set<string> tree::getDown(branch_t* branch, string value)
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

set<string> tree::getUp(branch_t* branch, string value)
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
