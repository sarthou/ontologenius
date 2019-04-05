#ifndef ONTOLOGENIUS_OBJECTROPERTYGRAPH_H
#define ONTOLOGENIUS_OBJECTROPERTYGRAPH_H

#include <string>
#include <vector>
#include <unordered_set>
#include <map>
#include <stdint.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/OntoGraph.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"

namespace ontologenius {

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

//for friend
class ObjectPropertyDrawer;
class IndividualGraph;

//for graphs usage
class ClassGraph;

class ObjectPropertyGraph : public OntoGraph<ObjectPropertyBranch_t>
{
  friend ObjectPropertyDrawer;
  friend IndividualGraph;
  friend ClassGraph;
public:
  ObjectPropertyGraph(ClassGraph* class_graph) {class_graph_ = class_graph; }
  ~ObjectPropertyGraph() {}

  void linkGraph(ClassGraph* class_graph) {class_graph_ = class_graph; }

  void add(std::string value, ObjectPropertyVectors_t& property_vectors);
  void add(std::vector<std::string>& disjoints);

  std::unordered_set<std::string> getDisjoint(const std::string& value);
  std::unordered_set<std::string> getInverse(const std::string& value);
  std::unordered_set<std::string> getDomain(const std::string& value);
  void getDomainPtr(ObjectPropertyBranch_t* branch, std::unordered_set<ClassBranch_t*>& res, size_t depth = -1);
  std::unordered_set<std::string> getRange(const std::string& value);
  void getRangePtr(ObjectPropertyBranch_t* branch, std::unordered_set<ClassBranch_t*>& res, size_t depth = -1);
  std::unordered_set<std::string> select(std::unordered_set<std::string>& on, const std::string& selector);

  void getDisjoint(ObjectPropertyBranch_t* branch, std::unordered_set<ObjectPropertyBranch_t*>& res);

  bool add(ObjectPropertyBranch_t* prop, std::string& relation, std::string& data);
  bool addInvert(ObjectPropertyBranch_t* prop, std::string& relation, std::string& data);
  bool remove(ObjectPropertyBranch_t* prop, std::string& relation, std::string& data);

private:
  ClassGraph* class_graph_;

  void isMyDisjoint(ObjectPropertyBranch_t* me, std::string& disjoint, std::map<std::string, ObjectPropertyBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    auto it = vect.find(disjoint);
    if(it != vect.end())
    {
      me->setSteady_disjoint(it->second);
      if(all)
        it->second->disjoints_.push_back(me);
      find = true;
    }
  }

  void isMyInverse(ObjectPropertyBranch_t* me, std::string& inverse, std::map<std::string, ObjectPropertyBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    auto it = vect.find(inverse);
    if(it != vect.end())
    {
      me->setSteady_inverse(it->second);
      if(all)
        it->second->inverses_.push_back(me);
      find = true;
    }
  }

  void isMyDomain(ObjectPropertyBranch_t* me, std::string& domain, std::map<std::string, ClassBranch_t*>& vect, bool& find)
  {
    if(find)
      return;

    auto it = vect.find(domain);
    if(it != vect.end())
    {
      me->setSteady_domain(it->second);
      find = true;
    }
  }

  void isMyRange(ObjectPropertyBranch_t* me, std::string& range, std::map<std::string, ClassBranch_t*>& vect, bool& find)
  {
    if(find)
      return;

    auto it = vect.find(range);
    if(it != vect.end())
    {
      me->setSteady_range(it->second);
      find = true;
    }
  }

  void getNextChainLink(ObjectPropertyBranch_t** next, std::string& next_link, std::map<std::string, ObjectPropertyBranch_t*>& vect)
  {
    if(*next == nullptr)
    {
      auto it = vect.find(next_link);
      if(it != vect.end())
        *next = it->second;
    }
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJECTROPERTYGRAPH_H
