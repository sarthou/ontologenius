#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdint.h>

#include "ontoloGenius/ontoGraphs/Graphs/OntoGraph.h"

#ifndef TREEOBJECT_H
#define TREEOBJECT_H

class ClassBranch_t : public Branch_t<ClassBranch_t>
{
public:
  std::vector<ClassBranch_t*> disjoints_;

  ClassBranch_t(std::string value) : Branch_t(value) {};
};

struct ObjectVectors_t
{
   std::vector<std::string> mothers_;
   std::vector<std::string> disjoints_;
   std::map<std::string, std::string> dictionary_;
};

class GraphDrawer;
class ObjectPropertyGraph;
class IndividualGraph;

class ClassGraph : public OntoGraph<ClassBranch_t>
{
  friend GraphDrawer;
  friend ObjectPropertyGraph;
  friend IndividualGraph;
public:
  ClassGraph() {}
  ~ClassGraph() {}

  void add(std::string value, ObjectVectors_t& object_vector);
  void add(std::vector<std::string>& disjoints);

  std::set<std::string> getDisjoint(std::string& value);
  std::set<std::string> select(std::set<std::string> on, std::string class_selector);

private:
  void isMyDisjoint(ClassBranch_t* me, std::string disjoint, std::vector<ClassBranch_t*>& vect, bool& find, bool all = true)
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
};

#endif /* TREEOBJECT_H */
