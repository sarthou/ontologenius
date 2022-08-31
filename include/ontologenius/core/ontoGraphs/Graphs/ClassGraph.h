#ifndef ONTOLOGENIUS_CLASSGRAPH_H
#define ONTOLOGENIUS_CLASSGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>

#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"

namespace ontologenius {

struct ObjectVectors_t
{
   std::vector<Single_t<std::string>> mothers_;
   std::vector<Single_t<std::string>> disjoints_;
   std::map<std::string, std::vector<std::string>> dictionary_;
   std::map<std::string, std::vector<std::string>> muted_dictionary_;

   std::vector<Pair_t<std::string, std::string>> object_relations_;
   std::vector<Pair_t<std::string, data_t>> data_relations_;
};

//for friend
class ClassDrawer;
class ObjectPropertyGraph;
class DataPropertyGraph;
class IndividualGraph;
class ClassChecker;

class ClassGraph : public OntoGraph<ClassBranch_t>
{
  friend ClassDrawer;
  friend ObjectPropertyGraph;
  friend DataPropertyGraph;
  friend IndividualGraph;

  friend ClassChecker;
public:
  ClassGraph(IndividualGraph* individual_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
  ClassGraph(const ClassGraph& other, IndividualGraph* individual_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
  ~ClassGraph() {}

  ClassBranch_t* add(const std::string& value, ObjectVectors_t& object_vector, bool direct_load = false);
  void add(std::vector<std::string>& disjoints);

  void deepCopy(const ClassGraph& other);

  std::unordered_set<std::string> getDisjoint(const std::string& value);
  void getDisjoint(ClassBranch_t* branch, std::unordered_set<ClassBranch_t*>& res);
  std::unordered_set<std::string> select(const std::unordered_set<std::string>& on, const std::string& class_selector);

  std::unordered_set<std::string> getRelationFrom(const std::string& _class, int depth = -1);  //C3
  std::unordered_set<std::string> getRelatedFrom(const std::string& property);     //C3
  std::unordered_set<std::string> getRelationOn(const std::string& _class, int depth = -1);    //C4
  std::unordered_set<std::string> getRelatedOn(const std::string& property);       //C3
  std::unordered_set<std::string> getRelationWith(const std::string& _class);  //C3
  std::unordered_set<std::string> getRelatedWith(const std::string& _class);   //C3
  std::unordered_set<std::string> getFrom(const std::string& param);
  std::unordered_set<std::string> getFrom(const std::string& _class, const std::string& property);
  std::unordered_set<std::string> getOn(const std::string& param);
  std::unordered_set<std::string> getOn(const std::string& _class, const std::string& property);
  std::unordered_set<std::string> getWith(const std::string& param, int depth = -1);
  std::unordered_set<std::string> getWith(const std::string& first_class, const std::string& second_class, int depth = -1);
  std::unordered_set<std::string> getDomainOf(const std::string& _class, int depth = -1);
  std::unordered_set<std::string> getRangeOf(const std::string& _class, int depth = -1);
  std::unordered_set<std::string> getDomainOf(ClassBranch_t* branch, int depth = -1);
  std::unordered_set<std::string> getRangeOf(ClassBranch_t* branch, int depth = -1);

  std::unordered_set<std::string> getDownIndividual(ClassBranch_t* branch);
  void getDownIndividual(ClassBranch_t* branch, std::unordered_set<std::string>& res);
  std::unordered_set<IndividualBranch_t*> getDownIndividualPtrSafe(ClassBranch_t* branch);
  void getDownIndividualPtrSafe(ClassBranch_t* branch, std::unordered_set<IndividualBranch_t*>& res);

  void deleteClass(ClassBranch_t* _class);
  int deleteRelationsOnClass(ClassBranch_t* _class, std::vector<ClassBranch_t*> vect);
  void addLang(const std::string& _class, std::string& lang, const std::string& name);
  void addInheritage(std::string& class_base, std::string& class_inherited);
  void addRelation(ClassBranch_t*, const std::string& property, const std::string& class_on);
  void addRelation(ClassBranch_t*, const std::string& property, const std::string& type, const std::string& data);
  void addRelationInvert(const std::string& class_from, const std::string& property, ClassBranch_t* class_on);
  void removeLang(std::string& indiv, std::string& lang, std::string& name);
  void removeInheritage(std::string& class_base, std::string& class_inherited);
  void removeRelation(const std::string& class_from, const std::string& property, const std::string& class_on);
  void removeRelation(const std::string& class_from, const std::string& property, const std::string& type, const std::string& data);

private:
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;
  IndividualGraph* individual_graph_;

  void addObjectRelation(ClassBranch_t* me, Pair_t<std::string, std::string>& relation);
  void addDataRelation(ClassBranch_t* me, Pair_t<std::string, data_t>& relation);

  void getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<std::string>& res, int depth);
  void getRelatedFrom(const std::unordered_set<uint32_t>& object_properties, const std::unordered_set<uint32_t>& data_properties, std::unordered_set<std::string>& res);
  void getRelationOnDataProperties(const std::string& _class, std::unordered_set<std::string>& res, int depth);
  void getRelatedOnDataProperties(const std::string& property, std::unordered_set<std::string>& res);
  void getRelationWith(ClassBranch_t* class_branch, std::map<std::string, int>& properties, std::vector<int>& depths, std::vector<std::string>& res, int depth);
  void dataGetRelatedWith(ClassBranch_t* class_branch, const std::string& property, const data_t& data, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake);
  void objectGetRelatedWith(ClassBranch_t* class_branch, const std::string& property, const std::string& _class, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake);
  void getOn(ClassBranch_t* class_branch, std::unordered_set<uint32_t>& object_properties, std::unordered_set<uint32_t>& data_properties, std::unordered_set<std::string>& res, uint32_t current_depth, int& found_depth);
  void getWith(ClassBranch_t* first_class, const std::string& second_class, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake, uint32_t current_depth, int& found_depth, int depth_prop, std::unordered_set<ClassBranch_t*>& next_step);

  bool checkRangeAndDomain(ClassBranch_t* from, ObjectPropertyBranch_t* prop, ClassBranch_t* on);
  bool checkRangeAndDomain(ClassBranch_t* from, DataPropertyBranch_t* prop, data_t& data);

  void isMyDisjoint(ClassBranch_t* me, const std::string& disjoint, std::map<std::string, ClassBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    auto it = vect.find(disjoint);
    if(it != vect.end())
    {
      me->disjoints_.emplace_back(it->second);
      if(all)
        it->second->disjoints_.emplace_back(me); // TODO do not save
      find = true;
    }
  }

  ClassBranch_t* findIntersection(const std::unordered_set<ClassBranch_t*>& base, const std::unordered_set<ClassBranch_t*>& comp)
  {
    for (ClassBranch_t* it : comp)
    {
      if(base.find(it) != base.end())
        return it;
    }
    return nullptr;
  }

  void cpyBranch(ClassBranch_t* old_branch, ClassBranch_t* new_branch);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSGRAPH_H
