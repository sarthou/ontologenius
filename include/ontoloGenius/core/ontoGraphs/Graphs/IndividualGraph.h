#ifndef ONTOLOGENIUS_INDIVIDUALGRAPH_H
#define ONTOLOGENIUS_INDIVIDUALGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/Graph.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/IndividualBranch.h"

namespace ontologenius {

struct IndividualVectors_t
{
   std::vector<Single_t<std::string>> is_a_;

   std::vector<Pair_t<std::string, std::string>> object_relations_;
   std::vector<Pair_t<std::string, data_t>> data_relations_;

   std::vector<Single_t<std::string>> same_as_;
   std::map<std::string, std::vector<std::string>> dictionary_;
   std::map<std::string, std::vector<std::string>> muted_dictionary_;
};

//for friend
class IndividualChecker;

//for graphs usage
class ClassGraph;
class ObjectPropertyGraph;
class DataPropertyGraph;

class IndividualGraph : public Graph<IndividualBranch_t>
{
  friend IndividualChecker;
public:
  IndividualGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
  IndividualGraph(const IndividualGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
  ~IndividualGraph();

  void deepCopy(const IndividualGraph& other);

  void linkGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);

  void close();
  std::vector<IndividualBranch_t*> get() {return individuals_; }

  std::vector<IndividualBranch_t*> getSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    return individuals_;
  }

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
  std::vector<std::string> getEveryNames(const std::string& value);
  std::unordered_set<std::string> find(const std::string& value);
  std::unordered_set<std::string> findSub(const std::string& value);
  std::unordered_set<std::string> findRegex(const std::string& regex);
  std::unordered_set<std::string> findFuzzy(const std::string& value, double threshold = 0.5);
  bool touch(const std::string& value);
  std::unordered_set<std::string> getType(const std::string& class_selector);

  ClassBranch_t* upgradeToBranch(IndividualBranch_t* indiv);
  void createIndividual(std::string& name);
  void deleteIndividual(IndividualBranch_t* indiv);
  void redirectDeleteIndividual(IndividualBranch_t* indiv, ClassBranch_t* _class);
  void addLang(std::string& indiv, std::string& lang, std::string& name);
  void addInheritage(std::string& indiv, std::string& class_inherited);
  void addInheritageInvert(std::string& indiv, std::string& class_inherited);
  void addInheritageInvertUpgrade(std::string& indiv, std::string& class_inherited);
  bool addProperty(std::string& indiv_from, std::string& property, std::string& indiv_on);
  bool addProperty(std::string& indiv_from, std::string& property, std::string& type, std::string& data);
  bool addPropertyInvert(std::string& indiv_from, std::string& property, std::string& indiv_on);
  void removeLang(std::string& indiv, std::string& lang, std::string& name);
  void removeInheritage(std::string& indiv, std::string& class_inherited);
  bool removeProperty(IndividualBranch_t* branch_from, ObjectPropertyBranch_t* property, IndividualBranch_t* branch_on);
  bool removeProperty(std::string& indiv_from, std::string& property, std::string& indiv_on);
  bool removeProperty(std::string& indiv_from, std::string& property, std::string& type, std::string& data);
  void removePropertyInverse(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);
  void removePropertySymetric(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);
  void removePropertyChain(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);
  std::vector<IndividualBranch_t*> resolveLink(std::vector<ObjectPropertyBranch_t*>& chain, IndividualBranch_t* indiv_on, size_t index);

  void getUpPtr(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*>& res, int depth = -1, unsigned int current_depth = 0);

private:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;

  std::vector<IndividualBranch_t*> individuals_;

  IndividualBranch_t* getBranch(const std::string& name)
  {
    for(size_t indiv_i = 0; indiv_i < individuals_.size(); indiv_i++)
      if(name == individuals_[indiv_i]->value())
        return individuals_[indiv_i];
    return nullptr;
  }

  void addObjectProperty(IndividualBranch_t* me, Pair_t<std::string, std::string>& relation);
  void addDataProperty(IndividualBranch_t* me, Pair_t<std::string, data_t>& relation);
  void setObjectPropertiesUpdated(std::vector<IndivObjectRelationElement_t>& relations);

  void getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<std::string>& res, int depth = -1);
  bool getRelatedWith(ClassBranch_t* class_branch, const std::string& data, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<uint32_t>& took);
  bool getFrom(ClassBranch_t* class_branch, std::unordered_set<uint32_t>& object_properties, std::unordered_set<uint32_t>& data_properties, const std::string& data, std::unordered_set<uint32_t>& down_classes, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<uint32_t>& doNotTake);

  std::unordered_set<uint32_t> getSameId(const std::string& individual);
  void getSame(IndividualBranch_t* individual, std::unordered_set<IndividualBranch_t*>& res);
  std::unordered_set<std::string> getSameAndClean(IndividualBranch_t* individual);
  std::unordered_set<uint32_t> getSameIdAndClean(IndividualBranch_t* individual);
  void cleanMarks(std::unordered_set<IndividualBranch_t*>& indSet);
  std::unordered_set<std::string> set2set(std::unordered_set<IndividualBranch_t*>& indSet, bool clean = true);

  bool checkRangeAndDomain(IndividualBranch_t* from, ObjectPropertyBranch_t* prop, IndividualBranch_t* on);
  bool checkRangeAndDomain(IndividualBranch_t* from, DataPropertyBranch_t* prop, data_t& data);

  void cpyBranch(IndividualBranch_t* old_branch, IndividualBranch_t* new_branch);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALGRAPH_H
