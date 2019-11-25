#ifndef ONTOLOGENIUS_DATAPROPERTYGRAPH_H
#define ONTOLOGENIUS_DATAPROPERTYGRAPH_H

#include <string>
#include <vector>
#include <unordered_set>
#include <map>
#include <stdint.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/OntoGraph.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"

namespace ontologenius {

struct DataPropertyVectors_t
{
   std::vector<Single_t<std::string>> mothers_;
   std::vector<Single_t<std::string>> disjoints_;
   std::vector<Single_t<std::string>> domains_;
   std::vector<std::string> ranges_;
   Properties_t properties_;
   std::map<std::string, std::vector<std::string>> dictionary_;
   std::map<std::string, std::vector<std::string>> muted_dictionary_;
};

//for friend
class DataPropertyDrawer;
class IndividualGraph;

//for graphs usage
class ClassGraph;

class DataPropertyGraph : public OntoGraph<DataPropertyBranch_t>
{
  friend DataPropertyDrawer;
  friend IndividualGraph;
  friend ClassGraph;
public:
  DataPropertyGraph(ClassGraph* class_graph);
  DataPropertyGraph(const DataPropertyGraph& other, ClassGraph* class_graph);
  ~DataPropertyGraph() {}

  void deepCopy(const DataPropertyGraph& other);

  void linkGraph(ClassGraph* class_graph) {class_graph_ = class_graph; }

  void add(std::string value, DataPropertyVectors_t& property_vectors);
  void add(std::vector<std::string>& disjoints);

  std::unordered_set<std::string> getDisjoint(const std::string& value);
  std::unordered_set<std::string> getDomain(const std::string& value);
  void getDomainPtr(DataPropertyBranch_t* branch, std::unordered_set<ClassBranch_t*>& res, size_t depth = -1);
  std::unordered_set<std::string> getRange(const std::string& value);
  std::unordered_set<std::string> select(std::unordered_set<std::string>& on, const std::string& selector);

  bool add(DataPropertyBranch_t* prop, std::string& relation, std::string& data);
  bool addInvert(DataPropertyBranch_t* prop, std::string& relation, std::string& data);
  bool remove(DataPropertyBranch_t* prop, std::string& relation, std::string& data);

private:
  ClassGraph* class_graph_;

  void isMyDisjoint(DataPropertyBranch_t* me, const std::string& disjoint, std::map<std::string, DataPropertyBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    auto it = vect.find(disjoint);
    if(it != vect.end())
    {
      me->disjoints_.push_back(it->second);
      if(all)
        it->second->disjoints_.push_back(me); // TODO do not save
      find = true;
    }
  }

  void cpyBranch(DataPropertyBranch_t* old_branch, DataPropertyBranch_t* new_branch);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTYGRAPH_H
