#include <string>
#include <vector>
#include <set>
#include <map>
#include <stdint.h>

#include "ontoloGenius/ontoGraphs/OntoGraph.h"
#include "ontoloGenius/ontoGraphs/ClassGraph.h"

#ifndef TREEPROPERTY_H
#define TREEPROPERTY_H

using namespace std;

struct PropertyClassBranch_t;

struct Properties_t
{
  bool functional_property_;
  bool inverse_functional_property_;
  bool transitive_property_;
  vector<PropertyClassBranch_t*> transitive_with_;
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

class PropertyClassBranch_t : public Branch_t<PropertyClassBranch_t>
{
public:
  vector<PropertyClassBranch_t*> disjoints_;
  vector<PropertyClassBranch_t*> inverses_;
  vector<ClassBranch_t*> domains_;
  vector<ClassBranch_t*> ranges_;
  Properties_t properties_;

  PropertyClassBranch_t(string value) : Branch_t(value) {};
};

struct PropertyVectors_t
{
   vector<string> mothers_;
   vector<string> disjoints_;
   vector<string> inverses_;
   vector<string> domains_;
   vector<string> ranges_;
   Properties_t properties_;
   map<string, string> dictionary_;
};

class GraphDrawer;

class PropertyGraph : public OntoGraph<PropertyClassBranch_t>
{
  friend GraphDrawer;
public:
  PropertyGraph(ClassGraph* treeObject) {treeObject_ = treeObject; }
  ~PropertyGraph() {}

  void add(string value, PropertyVectors_t& property_vectors);
  void add(vector<string>& disjoints);

  set<string> getDisjoint(string value);
  set<string> getInverse(string value);
  set<string> getDomain(string value);
  set<string> getRange(string value);

private:
  ClassGraph* treeObject_;
};

#endif /* TREEPROPERTY_H */
