#include <string>
#include <vector>
#include <set>
#include <stdint.h>

#include "ontoloGenius/treeObject.h"

#ifndef TREEPROPERTY_H
#define TREEPROPERTY_H

using namespace std;

struct propertyBranch_t;

struct properties_t
{
  bool functionalProperty;
  bool inverseFunctionalProperty;
  bool transitiveProperty;
  vector<propertyBranch_t*> transitiveWith;
  bool symetricProperty;
  bool antisymetricProperty;
  bool reflexiveProperty;
  bool irreflexiveProperty;

  properties_t() : functionalProperty(false),
                    inverseFunctionalProperty(false),
                    transitiveProperty(false),
                    symetricProperty(false),
                    antisymetricProperty(false),
                    reflexiveProperty(false),
                    irreflexiveProperty(false) {};
};

struct propertyBranch_t
{
  string value;
  vector<propertyBranch_t*> childs;
  vector<propertyBranch_t*> mothers;
  vector<propertyBranch_t*> disjoints;
  vector<propertyBranch_t*> inverses;
  vector<branch_t*> domains;
  vector<branch_t*> range;
  uint8_t family;
  uint8_t nb_mothers;
  properties_t properties;

  propertyBranch_t(string p_value) : family(0), nb_mothers(0)
    {value = p_value; };
};

class tree_drawer;

class treeProperty
{
  friend tree_drawer;
public:
  treeProperty() {}
  ~treeProperty();

  void add(string value, vector<string>& mothers, vector<string>& disjoints, vector<string>& inverses);
  void add(vector<string>& disjoints);
  void close();

  set<string> getDown(string value);
  set<string> getUp(string value);
  set<string> getDisjoint(string value);
  set<string> getInverse(string value);

private:
  vector<propertyBranch_t*> branchs;
  vector<propertyBranch_t*> roots;

  vector<propertyBranch_t*> tmp_mothers;

  int depth;

  void link();
  void add_family(propertyBranch_t* branch, uint8_t family);

  set<string> getDown(propertyBranch_t* branch, string value);
  set<string> getUp(propertyBranch_t* branch, string value);
};

#endif /* TREEPROPERTY_H */
