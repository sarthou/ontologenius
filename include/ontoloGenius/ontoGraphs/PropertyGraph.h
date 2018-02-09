#include <string>
#include <vector>
#include <set>
#include <map>
#include <stdint.h>

#include "ontoloGenius/ontoGraphs/OntoGraph.h"
#include "ontoloGenius/ontoGraphs/ClassGraph.h"

#ifndef TREEPROPERTY_H
#define TREEPROPERTY_H

struct PropertyClassBranch_t;

struct Properties_t
{
  bool functional_property_;
  bool inverse_functional_property_;
  bool transitive_property_;
  std::vector<PropertyClassBranch_t*> transitive_with_;
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
  std::vector<PropertyClassBranch_t*> disjoints_;
  std::vector<PropertyClassBranch_t*> inverses_;
  std::vector<ClassBranch_t*> domains_;
  std::vector<ClassBranch_t*> ranges_;
  Properties_t properties_;

  PropertyClassBranch_t(std::string value) : Branch_t(value) {};
};

struct PropertyVectors_t
{
   std::vector<std::string> mothers_;
   std::vector<std::string> disjoints_;
   std::vector<std::string> inverses_;
   std::vector<std::string> domains_;
   std::vector<std::string> ranges_;
   Properties_t properties_;
   std::map<std::string, std::string> dictionary_;
};

class GraphDrawer;

class PropertyGraph : public OntoGraph<PropertyClassBranch_t>
{
  friend GraphDrawer;
public:
  PropertyGraph(ClassGraph* treeObject) {treeObject_ = treeObject; }
  ~PropertyGraph() {}

  void add(std::string value, PropertyVectors_t& property_vectors);
  void add(std::vector<std::string>& disjoints);

  std::set<std::string> getDisjoint(std::string value);
  std::set<std::string> getInverse(std::string value);
  std::set<std::string> getDomain(std::string value);
  std::set<std::string> getRange(std::string value);

private:
  ClassGraph* treeObject_;
};

#endif /* TREEPROPERTY_H */
