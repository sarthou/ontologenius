#ifndef CLASSGRAPH_H
#define CLASSGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"

struct ObjectVectors_t
{
   std::vector<std::string> mothers_;
   std::vector<std::string> disjoints_;
   std::map<std::string, std::vector<std::string>> dictionary_;
};

class ClassDrawer;
class ObjectPropertyGraph;
class DataPropertyGraph;
class IndividualGraph;

class ClassGraph : public OntoGraph<ClassBranch_t>
{
  friend ClassDrawer;
  friend ObjectPropertyGraph;
  friend DataPropertyGraph;
  friend IndividualGraph;
public:
  ClassGraph() {}
  ~ClassGraph() {}

  void add(const std::string& value, ObjectVectors_t& object_vector);
  void add(std::vector<std::string>& disjoints);

  std::unordered_set<std::string> getDisjoint(const std::string& value);
  std::unordered_set<std::string> select(std::unordered_set<std::string>& on, const std::string& class_selector);

private:
  void isMyDisjoint(ClassBranch_t* me, const std::string& disjoint, std::vector<ClassBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    for(size_t i = 0; i < vect.size(); i++)
      if(disjoint == vect[i]->value())
      {
        me->setSteady_disjoint(vect[i]);
        if(all)
          vect[i]->disjoints_.push_back(me);
        find = true;
        break;
      }
  }
};

#endif /* CLASSGRAPH_H */
