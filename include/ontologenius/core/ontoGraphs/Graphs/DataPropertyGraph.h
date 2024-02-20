#ifndef ONTOLOGENIUS_DATAPROPERTYGRAPH_H
#define ONTOLOGENIUS_DATAPROPERTYGRAPH_H

#include <string>
#include <vector>
#include <unordered_set>
#include <map>
#include <stdint.h>

#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"

#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"

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
   bool annotation_usage_;

   DataPropertyVectors_t() : annotation_usage_(false) {}
};

//for friend
class DataPropertyDrawer;
class IndividualGraph;
class AnonymousClassGraph;

//for graphs usage
class ClassGraph;

class DataPropertyGraph : public OntoGraph<DataPropertyBranch_t>
{
  friend DataPropertyDrawer;
  friend IndividualGraph;
  friend ClassGraph;
  friend AnonymousClassGraph;
public:
  explicit DataPropertyGraph(IndividualGraph* individual_graph, ClassGraph* class_graph);
  DataPropertyGraph(const DataPropertyGraph& other, IndividualGraph* individual_graph, ClassGraph* class_graph);
  ~DataPropertyGraph() {}

  void deepCopy(const DataPropertyGraph& other);

  DataPropertyBranch_t* add(const std::string& value, DataPropertyVectors_t& property_vectors);
  void add(std::vector<std::string>& disjoints);
  bool addAnnotation(const std::string& value, DataPropertyVectors_t& property_vectors);
  LiteralNode* createLiteral(const std::string& value);
  LiteralNode* createLiteralUnsafe(const std::string& value);

  std::unordered_set<std::string> getDomain(const std::string& value, size_t depth = -1);
  std::unordered_set<index_t> getDomain(index_t value, size_t depth = -1);
  void getDomainPtr(DataPropertyBranch_t* branch, std::unordered_set<ClassBranch_t*>& res, size_t depth = -1);
  std::unordered_set<std::string> getRange(const std::string& value);
  std::unordered_set<index_t> getRange(index_t value);

  index_t getLiteralIndex(const std::string& name);
  std::vector<index_t> getLiteralIndexes(const std::vector<std::string>& names);
  std::string getLiteralIdentifier(index_t index);
  std::vector<std::string> getLiteralIdentifiers(const std::vector<index_t>& indexes);

private:
  ClassGraph* class_graph_;
  BranchContainerSet<LiteralNode> literal_container_;

  template<typename T> void getDomain(DataPropertyBranch_t* branch, size_t depth, std::unordered_set<T>& res, std::unordered_set<DataPropertyBranch_t*>& up_trace);
  void getDomainPtr(DataPropertyBranch_t* branch, size_t depth, std::unordered_set<ClassBranch_t*>& res, std::unordered_set<DataPropertyBranch_t*>& up_trace);

  void cpyBranch(DataPropertyBranch_t* old_branch, DataPropertyBranch_t* new_branch);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTYGRAPH_H
