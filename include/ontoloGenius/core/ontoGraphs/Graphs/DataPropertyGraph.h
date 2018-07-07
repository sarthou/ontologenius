#include <string>
#include <vector>
#include <unordered_set>
#include <map>
#include <stdint.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

#ifndef DATAPROPERTYGRAPH_H
#define DATAPROPERTYGRAPH_H

struct DataPropertyVectors_t
{
   std::vector<std::string> mothers_;
   std::vector<std::string> disjoints_;
   std::vector<std::string> domains_;
   std::vector<std::string> ranges_;
   Properties_t properties_;
   std::map<std::string, std::vector<std::string>> dictionary_;
};

class DataPropertyDrawer;
class IndividualGraph;

class DataPropertyGraph : public OntoGraph<DataPropertyBranch_t>
{
  friend DataPropertyDrawer;
  friend IndividualGraph;
public:
  DataPropertyGraph(ClassGraph* class_graph) {class_graph_ = class_graph; }
  ~DataPropertyGraph() {}

  void add(std::string value, DataPropertyVectors_t& property_vectors);
  void add(std::vector<std::string>& disjoints);

  std::unordered_set<std::string> getDisjoint(const std::string& value);
  std::unordered_set<std::string> getDomain(const std::string& value);
  std::unordered_set<std::string> getRange(const std::string& value);
  std::unordered_set<std::string> select(std::unordered_set<std::string>& on, const std::string& selector);

private:
  ClassGraph* class_graph_;

  void isMyDisjoint(DataPropertyBranch_t* me, std::string disjoint, std::vector<DataPropertyBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    for(unsigned int i = 0; i < vect.size(); i++)
      if(disjoint == vect[i]->value_)
      {
        me->setSteady_disjoint(vect[i]);
        if(all)
          vect[i]->disjoints_.push_back(me);
        find = true;
        break;
      }
  }

  void isMyDomain(DataPropertyBranch_t* me, std::string domain, std::vector<ClassBranch_t*>& vect, bool& find)
  {
    if(find)
      return;

    for(unsigned int i = 0; i < vect.size(); i++)
      if(domain == vect[i]->value_)
      {
        me->setSteady_domain(vect[i]);
        find = true;
        break;
      }
  }
};

#endif /* DATAPROPERTYGRAPH_H */
