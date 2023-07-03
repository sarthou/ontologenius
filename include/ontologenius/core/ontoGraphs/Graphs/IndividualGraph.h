#ifndef ONTOLOGENIUS_INDIVIDUALGRAPH_H
#define ONTOLOGENIUS_INDIVIDUALGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>

#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"

#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"

namespace ontologenius {

struct IndividualVectors_t
{
   std::vector<Single_t<std::string>> is_a_;

   std::vector<Pair_t<std::string, std::string>> object_relations_;
   std::vector<Pair_t<std::string, std::string>> data_relations_;

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

  void close() final;
  std::vector<IndividualBranch_t*> get() override {return individuals_; }

  std::vector<IndividualBranch_t*> getSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    return individuals_;
  }

  std::vector<std::string> getAll()
  {
    std::vector<std::string> res;
    std::transform(individuals_.cbegin(), individuals_.cend(), std::back_inserter(res), [](auto branch){ return branch->value(); });
    return res;
  }

  IndividualBranch_t* add(const std::string& value, IndividualVectors_t& individual_vector);
  void add(std::vector<std::string>& distinct_);

  std::unordered_set<std::string> getSame(const std::string& individual);          //C1
  std::unordered_set<index_t> getSame(index_t individual);
  std::unordered_set<std::string> getDistincts(const std::string& individual);     //C2
  std::unordered_set<index_t> getDistincts(index_t individual);
  std::unordered_set<std::string> getRelationFrom(const std::string& individual, int depth = -1);  //C3
  std::unordered_set<index_t> getRelationFrom(index_t individual, int depth);
  std::unordered_set<std::string> getRelatedFrom(const std::string& property);     //C3
  std::unordered_set<index_t> getRelatedFrom(index_t property);
  std::unordered_set<std::string> getRelationOn(const std::string& individual, int depth = -1);    //C4
  std::unordered_set<index_t> getRelationOn(index_t individual, int depth = -1);
  std::unordered_set<std::string> getRelatedOn(const std::string& property);       //C3
  std::unordered_set<index_t> getRelatedOn(index_t property);
  std::unordered_set<std::string> getRelationWith(const std::string& individual);  //C3
  std::unordered_set<index_t> getRelationWith(index_t individual);
  std::unordered_set<std::string> getRelatedWith(const std::string& individual);   //C3
  std::unordered_set<index_t> getRelatedWith(index_t individual);
  std::unordered_set<std::string> getFrom(const std::string& param);
  std::unordered_set<std::string> getFrom(const std::string& individual, const std::string& property);
  std::unordered_set<std::string> getOn(const std::string& param);
  std::unordered_set<std::string> getOn(const std::string& individual, const std::string& property);
  std::unordered_set<std::string> getWith(const std::string& param, int depth = -1);
  std::unordered_set<std::string> getWith(const std::string& first_individual, const std::string& second_individual, int depth = -1);
  std::unordered_set<std::string> getDomainOf(const std::string& individual, int depth = -1);
  std::unordered_set<std::string> getRangeOf(const std::string& individual, int depth = -1);
  std::unordered_set<std::string> getUp(IndividualBranch_t* indiv, int depth = -1, uint32_t current_depth = 0);
  std::unordered_set<std::string> getUp(const std::string& individual, int depth = -1);            //C3
  std::unordered_set<std::string> select(const std::unordered_set<std::string>& on, const std::string& class_selector);
  std::string getName(const std::string& value, bool use_default = true);
  std::vector<std::string> getNames(const std::string& value, bool use_default = true);
  std::vector<std::string> getEveryNames(const std::string& value, bool use_default = true);
  std::unordered_set<std::string> find(const std::string& value, bool use_default = true);
  std::unordered_set<std::string> findSub(const std::string& value, bool use_default = true);
  std::unordered_set<std::string> findRegex(const std::string& regex, bool use_default = true);
  std::unordered_set<std::string> findFuzzy(const std::string& value, bool use_default = true, double threshold = 0.5);
  bool touch(const std::string& value);
  std::unordered_set<std::string> getType(const std::string& class_selector);
  bool relationExists(const std::string& param);
  bool relationExists(const std::string& subject, const std::string& property, const std::string& object);

  ClassBranch_t* upgradeToBranch(IndividualBranch_t* indiv);
  IndividualBranch_t* createIndividual(const std::string& name);
  IndividualBranch_t* createIndividualUnsafe(const std::string& name);
  void deleteIndividual(IndividualBranch_t* indiv);
  void redirectDeleteIndividual(IndividualBranch_t* indiv, ClassBranch_t* _class);
  bool addLang(const std::string& indiv, const std::string& lang, const std::string& name);
  bool addInheritage(const std::string& indiv, const std::string& class_inherited);
  bool addInheritage(IndividualBranch_t* branch, const std::string& class_inherited);
  bool addInheritageUnsafe(IndividualBranch_t* branch, const std::string& class_inherited);
  bool addInheritageInvert(const std::string& indiv, const std::string& class_inherited);
  bool addInheritageInvertUpgrade(const std::string& indiv, const std::string& class_inherited);
  int addRelation(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on, double proba = 1.0, bool infered = false);
  int addRelation(IndividualBranch_t* indiv_from, DataPropertyBranch_t* property, LiteralNode* data, double proba = 1.0, bool infered = false);
  void addRelation(IndividualBranch_t* indiv_from, const std::string& property, const std::string& indiv_on);
  void addRelation(IndividualBranch_t* indiv_from, const std::string& property, const std::string& type, const std::string& data);
  void addRelationInvert(const std::string& indiv_from, const std::string& property, IndividualBranch_t* indiv_on);
  bool removeLang(const std::string& indiv, const std::string& lang, const std::string& name);
  bool removeInheritage(const std::string& indiv, const std::string& class_inherited);
  bool addSameAs(const std::string& indiv_1, const std::string& indiv_2);
  bool removeSameAs(const std::string& indiv_1, const std::string& indiv_2);
  // removing a relation using an object property has to generate an "explanation" if it remove other relations
  std::vector<std::pair<std::string, std::string>> removeRelation(IndividualBranch_t* branch_from, ObjectPropertyBranch_t* property, IndividualBranch_t* branch_on, bool protect_infered = false);
  std::vector<std::pair<std::string, std::string>> removeRelation(const std::string& indiv_from, const std::string& property, const std::string& indiv_on);
  void removeRelation(const std::string& indiv_from, const std::string& property, const std::string& type, const std::string& data);
  std::vector<std::pair<std::string, std::string>> removeRelationInverse(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);
  std::vector<std::pair<std::string, std::string>> removeRelationSymetric(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);
  std::vector<std::pair<std::string, std::string>> removeRelationChain(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);
  std::vector<IndividualBranch_t*> resolveLink(std::vector<ObjectPropertyBranch_t*>& chain, IndividualBranch_t* indiv_on, size_t index);
  std::vector<std::vector<ObjectPropertyBranch_t*>> getChains(ObjectPropertyBranch_t* base_property);

  void getUpPtr(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*>& res, int depth = -1, uint32_t current_depth = 0);
  void getSameAndClean(IndividualBranch_t* individual, std::unordered_set<std::string>& res);
  void getSameAndClean(IndividualBranch_t* individual, std::unordered_set<index_t>& res);

private:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;

  std::vector<IndividualBranch_t*> individuals_;

  IndividualBranch_t* getBranch(const std::string& name) const
  {
    auto indiv_it = std::find_if(individuals_.begin(), individuals_.end(), [&name](IndividualBranch_t* branch){
      return branch->value() == name;
    });
    if(indiv_it != individuals_.end())
      return *indiv_it;
    else
      return nullptr;
  }

  template<typename T> std::unordered_set<T> getDistincts(IndividualBranch_t* individual);
  template<typename T> std::unordered_set<T> getRelationFrom(IndividualBranch_t* individual, int depth);
  template<typename T> std::unordered_set<T> getRelatedFrom(const T& property);
  template<typename T> void getRelatedOn(const T& property, std::unordered_set<T>& res);
  template<typename T> void getUp(IndividualBranch_t* indiv, std::unordered_set<T>& res, int depth = -1, uint32_t current_depth = 0);
  template<typename T> void getRelatedWith(index_t individual, std::unordered_set<T>& res);

  void addObjectRelation(IndividualBranch_t* me, Pair_t<std::string, std::string>& relation);
  void addDataRelation(IndividualBranch_t* me, Pair_t<std::string, std::string>& relation);

  template<typename T> void getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<T>& res, int depth = -1);
  bool getRelatedWith(ClassBranch_t* class_branch, index_t data, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<index_t>& took);
  bool getFrom(ClassBranch_t* class_branch, const std::unordered_set<index_t>& object_properties, const std::unordered_set<index_t>& data_properties, LiteralNode* data, const std::unordered_set<index_t>& down_classes, std::unordered_set<ClassBranch_t*>& next_step, std::unordered_set<index_t>& do_not_take);

  bool relationExists(IndividualBranch_t* subject, ObjectPropertyBranch_t* property, IndividualBranch_t* object);

  void getDistincts(IndividualBranch_t* individual, std::unordered_set<IndividualBranch_t*>& res);
  std::unordered_set<index_t> getSameId(const std::string& individual);
  std::unordered_set<index_t> getSameId(index_t individual);
  void getSame(IndividualBranch_t* individual, std::unordered_set<IndividualBranch_t*>& res);
  std::unordered_set<std::string> getSameAndClean(IndividualBranch_t* individual);
  std::unordered_set<index_t> getSameIdAndClean(IndividualBranch_t* individual);
  void cleanMarks(const std::unordered_set<IndividualBranch_t*>& indiv_set);
  std::unordered_set<std::string> set2set(const std::unordered_set<IndividualBranch_t*>& indiv_set, bool clean = true);

  bool checkRangeAndDomain(IndividualBranch_t* from, ObjectPropertyBranch_t* prop, IndividualBranch_t* on);
  bool checkRangeAndDomain(IndividualBranch_t* from, DataPropertyBranch_t* prop, LiteralNode* data);

  void cpyBranch(IndividualBranch_t* old_branch, IndividualBranch_t* new_branch);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALGRAPH_H
