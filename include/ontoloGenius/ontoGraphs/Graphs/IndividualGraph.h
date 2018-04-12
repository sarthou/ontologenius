#ifndef INDIVIDUALGRAPH_H
#define INDIVIDUALGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdint.h>

#include "ontoloGenius/ontoGraphs/Graphs/Graph.h"
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
  std::map<std::string, std::string> dictionary_;

  bool mark;

  IndividualBranch_t(std::string value) : ValuedNode(value) {mark = false; }
};

struct IndividualVectors_t
{
   std::vector<std::string> is_a_;
   std::vector<std::string> properties_name_;
   std::vector<std::string> properties_on_;
   std::vector<std::string> same_as_;
   std::map<std::string, std::string> dictionary_;
};

class IndividualChecker;

class IndividualGraph : public Graph<IndividualBranch_t>
{
  friend IndividualChecker;
public:
  IndividualGraph(ClassGraph* classes, PropertyGraph* properties);
  ~IndividualGraph();

  void close();
  std::vector<IndividualBranch_t*> get() {return individuals_; }

  void add(std::string value, IndividualVectors_t& individual_vector);
  void add(std::vector<std::string>& distinct_);

  std::set<std::string> getSame(std::string individual);          //C1
  std::set<std::string> getDistincts(std::string individual);     //C2
  std::set<std::string> getRelationFrom(std::string individual);  //C3
  std::set<std::string> getRelatedFrom(std::string property);     //C3
  std::set<std::string> getRelationOn(std::string individual);    //C4
  std::set<std::string> getRelatedOn(std::string property);       //C3
  std::set<std::string> getRelationWith(std::string individual);  //C3
  std::set<std::string> getRelatedWith(std::string individual);   //C3
  std::set<std::string> getFrom(std::string param);
  std::set<std::string> getFrom(std::string individual, std::string property);
  std::set<std::string> getOn(std::string param);
  std::set<std::string> getOn(std::string individual, std::string property);
  std::set<std::string> getUp(std::string individual);            //C3
  std::set<std::string> select(std::set<std::string> on, std::string class_selector);

private:
  ClassGraph* classes_;
  PropertyGraph* properties_;

  std::vector<IndividualBranch_t*> individuals_;

  std::set<IndividualBranch_t*> getSame(IndividualBranch_t* individual);
  void cleanMarks(std::set<IndividualBranch_t*>& indSet);
  std::set<std::string> set2set(std::set<IndividualBranch_t*> indSet, bool clean = true);
};

#endif /* INDIVIDUALGRAPH_H */
