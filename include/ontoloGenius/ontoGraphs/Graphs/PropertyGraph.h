#include <string>
#include <vector>
#include <set>
#include <map>
#include <stdint.h>

#include "ontoloGenius/ontoGraphs/Graphs/OntoGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"

#ifndef TREEPROPERTY_H
#define TREEPROPERTY_H

struct PropertyClassBranch_t;

struct Properties_t
{
  bool functional_property_;
  bool inverse_functional_property_;
  bool transitive_property_;
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
   std::vector<std::vector<std::string>> chains_;
   Properties_t properties_;
   std::map<std::string, std::string> dictionary_;
};

class GraphDrawer;
class IndividualGraph;

class PropertyGraph : public OntoGraph<PropertyClassBranch_t>
{
  friend GraphDrawer;
  friend IndividualGraph;
public:
  PropertyGraph(ClassGraph* treeObject) {treeObject_ = treeObject; }
  ~PropertyGraph() {}

  void add(std::string value, PropertyVectors_t& property_vectors);
  void add(std::vector<std::string>& disjoints);

  std::set<std::string> getDisjoint(std::string& value);
  std::set<std::string> getInverse(std::string& value);
  std::set<std::string> getDomain(std::string& value);
  std::set<std::string> getRange(std::string& value);
  std::set<std::string> select(std::set<std::string> on, std::string selector);

private:
  ClassGraph* treeObject_;

  void isMyDisjoint(PropertyClassBranch_t* me, std::string disjoint, std::vector<PropertyClassBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    for(unsigned int i = 0; i < vect.size(); i++)
      if(disjoint == vect[i]->value_)
      {
        me->disjoints_.push_back(vect[i]);
        if(all)
          vect[i]->disjoints_.push_back(me);
        find = true;
        break;
      }
  }

  void isMyInverse(PropertyClassBranch_t* me, std::string inverse, std::vector<PropertyClassBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    for(unsigned int i = 0; i < vect.size(); i++)
      if(inverse == vect[i]->value_)
      {
        me->inverses_.push_back(vect[i]);
        if(all)
          vect[i]->inverses_.push_back(me);
        find = true;
        break;
      }
  }

  void isMyDomain(PropertyClassBranch_t* me, std::string domain, std::vector<ClassBranch_t*>& vect, bool& find)
  {
    if(find)
      return;

    for(unsigned int i = 0; i < vect.size(); i++)
      if(domain == vect[i]->value_)
      {
        me->domains_.push_back(vect[i]);
        find = true;
        break;
      }
  }

  void isMyRange(PropertyClassBranch_t* me, std::string range, std::vector<ClassBranch_t*>& vect, bool& find)
  {
    if(find)
      return;

    for(unsigned int i = 0; i < vect.size(); i++)
      if(range == vect[i]->value_)
      {
        me->ranges_.push_back(vect[i]);
        find = true;
        break;
      }
  }
};

#endif /* TREEPROPERTY_H */
