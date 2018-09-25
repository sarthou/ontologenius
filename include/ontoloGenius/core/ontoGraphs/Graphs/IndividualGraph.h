#ifndef INDIVIDUALGRAPH_H
#define INDIVIDUALGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/IndividualBranch.h"

struct IndividualVectors_t
{
   std::vector<std::string> is_a_;

   std::vector<std::string> object_properties_name_;
   std::vector<std::string> object_properties_on_;

   std::vector<std::string> data_properties_name_;
   std::vector<std::string> data_properties_type_;
   std::vector<std::string> data_properties_value_;

   std::vector<std::string> same_as_;
   std::map<std::string, std::vector<std::string>> dictionary_;
};

class IndividualChecker;

class IndividualGraph : public Graph<IndividualBranch_t>
{
  friend IndividualChecker;
public:
  IndividualGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
  ~IndividualGraph();

  void close();
  std::vector<IndividualBranch_t*> get() {return individuals_; }

  void add(std::string value, IndividualVectors_t& individual_vector);
  void add(std::vector<std::string>& distinct_);

  std::unordered_set<std::string> getSame(const std::string& individual);          //C1
  std::unordered_set<std::string> getDistincts(const std::string& individual);     //C2
  std::unordered_set<std::string> getRelationFrom(const std::string& individual, int depth = -1);  //C3
  std::unordered_set<std::string> getRelatedFrom(const std::string& property);     //C3
  std::unordered_set<std::string> getRelationOn(const std::string& individual, int depth = -1);    //C4
  std::unordered_set<std::string> getRelatedOn(const std::string& property);       //C3
  std::unordered_set<std::string> getRelationWith(const std::string& individual);  //C3
  std::unordered_set<std::string> getRelatedWith(const std::string& individual);   //C3
  std::unordered_set<std::string> getFrom(const std::string& param);
  std::unordered_set<std::string> getFrom(const std::string& individual, const std::string& property);
  std::unordered_set<std::string> getOn(const std::string& param);
  std::unordered_set<std::string> getOn(const std::string& individual, const std::string& property);
  std::unordered_set<std::string> getWith(const std::string& param, int depth = -1);
  std::unordered_set<std::string> getWith(const std::string& first_individual, const std::string& second_individual, int depth = -1);
  std::unordered_set<std::string> getUp(IndividualBranch_t* indiv, int depth = -1, unsigned int current_depth = 0);
  std::unordered_set<std::string> getUp(const std::string& individual, int depth = -1);            //C3
  std::unordered_set<std::string> select(std::unordered_set<std::string>& on, const std::string& class_selector);
  std::string getName(const std::string& value);
  std::vector<std::string> getNames(const std::string& value);
  std::unordered_set<std::string> find(const std::string& value);
  std::unordered_set<std::string> getType(const std::string& class_selector);

private:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;

  std::vector<IndividualBranch_t*> individuals_;

  std::unordered_set<uint32_t> getSameId(const std::string& individual);
  void getSame(IndividualBranch_t* individual, std::unordered_set<IndividualBranch_t*>& res);
  std::unordered_set<std::string> getSameAndClean(IndividualBranch_t* individual);
  std::unordered_set<uint32_t> getSameIdAndClean(IndividualBranch_t* individual);
  void cleanMarks(std::unordered_set<IndividualBranch_t*>& indSet);
  std::unordered_set<std::string> set2set(std::unordered_set<IndividualBranch_t*>& indSet, bool clean = true);
};

#endif /* INDIVIDUALGRAPH_H */
