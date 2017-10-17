#include <string>
#include <vector>
#include <set>
#include <stdint.h>

#include "ontoloGenius/TreeObject.h"

#ifndef TREEPROPERTY_H
#define TREEPROPERTY_H

using namespace std;

struct PropertyBranch_t;

struct Properties_t
{
  bool functional_property_;
  bool inverse_functional_property_;
  bool transitive_property_;
  vector<PropertyBranch_t*> transitive_with_;
  bool symetric_property_;
  bool antisymetric_property_;
  bool reflexive_property_;
  bool irreflexive_property_;

  Properties_t() : functional_property_(false),
                    inverse_functional_property_(false),
                    transitive_property_(false),
                    symetric_property_(false),
                    antisymetric_property_(false),
                    reflexive_property_(false),
                    irreflexive_property_(false) {};
};

struct PropertyBranch_t
{
  string value_;
  vector<PropertyBranch_t*> childs_;
  vector<PropertyBranch_t*> mothers_;
  vector<PropertyBranch_t*> disjoints_;
  vector<PropertyBranch_t*> inverses_;
  vector<Branch_t*> domains_;
  vector<Branch_t*> ranges_;
  uint8_t family;
  uint8_t nb_mothers_;
  Properties_t properties_;

  PropertyBranch_t(string value) : family(0), nb_mothers_(0)
    {value_ = value; };
};

struct PropertyVectors_t
{
   vector<string> mothers_;
   vector<string> disjoints_;
   vector<string> inverses_;
   vector<string> domains_;
   vector<string> ranges_;
};

class TreeDrawer;

class TreeProperty
{
  friend TreeDrawer;
public:
  TreeProperty(TreeObject* treeObject) {treeObject_ = treeObject; }
  ~TreeProperty();

  void add(string value, PropertyVectors_t& property_vectors);
  void add(vector<string>& disjoints);
  void close();

  set<string> getDown(string value);
  set<string> getUp(string value);
  set<string> getDisjoint(string value);
  set<string> getInverse(string value);
  set<string> getDomain(string value);
  set<string> getRange(string value);

private:
  vector<PropertyBranch_t*> branchs_;
  vector<PropertyBranch_t*> roots_;

  vector<PropertyBranch_t*> tmp_mothers_;

  int depth_;

  TreeObject* treeObject_;

  void link();
  void add_family(PropertyBranch_t* property_branch, uint8_t family);

  set<string> getDown(PropertyBranch_t* property_branch, string value);
  set<string> getUp(PropertyBranch_t* property_branch, string value);
};

#endif /* TREEPROPERTY_H */
