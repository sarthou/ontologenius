#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdint.h>

#include "ontoloGenius/ontoGraphs/OntoGraph.h"

#ifndef TREEOBJECT_H
#define TREEOBJECT_H

using namespace std;

class ClassBranch_t : public Branch_t<ClassBranch_t>
{
public:
  std::vector<ClassBranch_t*> disjoints_;

  ClassBranch_t(std::string value) : Branch_t(value) {};
};

struct ObjectVectors_t
{
   vector<string> mothers_;
   vector<string> disjoints_;
   map<string, string> dictionary_;
};

class GraphDrawer;
class PropertyGraph;

class ClassGraph : public OntoGraph<ClassBranch_t>
{
  friend GraphDrawer;
  friend PropertyGraph;
public:
  ClassGraph() {}
  ~ClassGraph() {}

  void add(string value, ObjectVectors_t& object_vector);
  void add(vector<string>& disjoints);

  set<string> getDisjoint(string value);

private:
};

#endif /* TREEOBJECT_H */
