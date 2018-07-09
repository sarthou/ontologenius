#ifndef OBJECTROPERTYGRAPH_H
#define OBJECTROPERTYGRAPH_H

#include <string>
#include <vector>
#include <unordered_set>
#include <map>
#include <stdint.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"

struct ObjectPropertyVectors_t
{
   std::vector<std::string> mothers_;
   std::vector<std::string> disjoints_;
   std::vector<std::string> inverses_;
   std::vector<std::string> domains_;
   std::vector<std::string> ranges_;
   std::vector<std::vector<std::string>> chains_;
   Properties_t properties_;
   std::map<std::string, std::vector<std::string>> dictionary_;
};

class ObjectPropertyDrawer;
class IndividualGraph;

class ObjectPropertyGraph : public OntoGraph<ObjectPropertyBranch_t>
{
  friend ObjectPropertyDrawer;
  friend IndividualGraph;
public:
  ObjectPropertyGraph(ClassGraph* class_graph) {class_graph_ = class_graph; }
  ~ObjectPropertyGraph() {}

  void add(std::string value, ObjectPropertyVectors_t& property_vectors);
  void add(std::vector<std::string>& disjoints);

  std::unordered_set<std::string> getDisjoint(const std::string& value);
  std::unordered_set<std::string> getInverse(const std::string& value);
  std::unordered_set<std::string> getDomain(const std::string& value);
  std::unordered_set<std::string> getRange(const std::string& value);
  std::unordered_set<std::string> select(std::unordered_set<std::string>& on, const std::string& selector);

private:
  ClassGraph* class_graph_;

  void isMyDisjoint(ObjectPropertyBranch_t* me, std::string& disjoint, std::vector<ObjectPropertyBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    for(size_t i = 0; i < vect.size(); i++)
      if(disjoint == vect[i]->value_)
      {
        me->setSteady_disjoint(vect[i]);
        if(all)
          vect[i]->disjoints_.push_back(me);
        find = true;
        break;
      }
  }

  void isMyInverse(ObjectPropertyBranch_t* me, std::string& inverse, std::vector<ObjectPropertyBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    for(size_t i = 0; i < vect.size(); i++)
      if(inverse == vect[i]->value_)
      {
        me->setSteady_inverse(vect[i]);
        if(all)
          vect[i]->inverses_.push_back(me);
        find = true;
        break;
      }
  }

  void isMyDomain(ObjectPropertyBranch_t* me, std::string& domain, std::vector<ClassBranch_t*>& vect, bool& find)
  {
    if(find)
      return;

    for(size_t i = 0; i < vect.size(); i++)
      if(domain == vect[i]->value_)
      {
        me->setSteady_domain(vect[i]);
        find = true;
        break;
      }
  }

  void isMyRange(ObjectPropertyBranch_t* me, std::string& range, std::vector<ClassBranch_t*>& vect, bool& find)
  {
    if(find)
      return;

    for(size_t i = 0; i < vect.size(); i++)
      if(range == vect[i]->value_)
      {
        me->setSteady_range(vect[i]);
        find = true;
        break;
      }
  }

  void getNextChainLink(ObjectPropertyBranch_t** next, std::string& next_link, std::vector<ObjectPropertyBranch_t*>& vect)
  {
    if(*next == nullptr)
      for(size_t i = 0; i < vect.size(); i++)
      {
        if(vect[i]->value_ == next_link)
        {
          *next = vect[i];
          break;
        }
      }
  }
};

#endif /* OBJECTROPERTYGRAPH_H */
