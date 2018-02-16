#ifndef INDIVIDUALGRAPH_H
#define INDIVIDUALGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdint.h>

#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/PropertyGraph.h"

class IndividualBranch_t : public ValuedNode
{
public:
  std::vector<ClassBranch_t*> is_a_;
  std::vector<PropertyClassBranch_t*> properties_name_;
  std::vector<IndividualBranch_t*> properties_on_;
  std::vector<IndividualBranch_t*> same_as_;
  std::vector<IndividualBranch_t*> distinct_;

  bool mark;

  IndividualBranch_t(std::string value) : ValuedNode(value) {mark = false; }
};

struct IndividualVectors_t
{
   std::vector<std::string> is_a_;
   std::vector<std::string> properties_name_;
   std::vector<std::string> properties_on_;
   std::vector<std::string> same_as_;
};

class IndividualGraph
{
public:
  IndividualGraph(ClassGraph* classes, PropertyGraph* properties);
  ~IndividualGraph();

  void close();

  void add(std::string value, IndividualVectors_t& individual_vector);
  void add(std::vector<std::string>& distinct_);

  std::set<std::string> getSame(std::string individual);
  std::set<std::string> getRelationFrom(std::string individual);
  std::set<std::string> getRelatedWith(std::string individual);
  std::set<std::string> getUp(std::string individual);

private:
  ClassGraph* classes_;
  PropertyGraph* properties_;

  std::vector<IndividualBranch_t*> individuals_;
  BranchContainerMap<IndividualBranch_t> container_;

  std::set<IndividualBranch_t*> getSame(IndividualBranch_t* individual);
  void cleanMarks(std::set<IndividualBranch_t*>& indSet);
};

#endif /* INDIVIDUALGRAPH_H */
