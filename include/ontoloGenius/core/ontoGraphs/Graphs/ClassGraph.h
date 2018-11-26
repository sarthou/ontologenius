#ifndef CLASSGRAPH_H
#define CLASSGRAPH_H

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <stdint.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"

struct ObjectVectors_t
{
   std::vector<std::string> mothers_;
   std::vector<std::string> disjoints_;
   std::map<std::string, std::vector<std::string>> dictionary_;

   std::vector<std::string> object_properties_name_;
   std::vector<std::string> object_properties_on_;

   std::vector<std::string> data_properties_name_;
   std::vector<std::string> data_properties_type_;
   std::vector<std::string> data_properties_value_;
};

//for friend
class ClassDrawer;
class ObjectPropertyGraph;
class DataPropertyGraph;
class IndividualGraph;

class ClassGraph : public OntoGraph<ClassBranch_t>
{
  friend ClassDrawer;
  friend ObjectPropertyGraph;
  friend DataPropertyGraph;
  friend IndividualGraph;
public:
  ClassGraph(ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
  ClassGraph(const ClassGraph& base) : OntoGraph<ClassBranch_t>(base) {}
  ~ClassGraph() {}

  void add(const std::string& value, ObjectVectors_t& object_vector);
  void add(std::vector<std::string>& disjoints);

  std::unordered_set<std::string> getDisjoint(const std::string& value);
  std::unordered_set<std::string> select(std::unordered_set<std::string>& on, const std::string& class_selector);

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

private:
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;

  void getRelationFrom(ClassBranch_t* class_branch, std::unordered_set<std::string>& res, int depth);
  void getRelatedFrom(std::unordered_set<uint32_t>& object_properties, std::unordered_set<uint32_t>& data_properties, std::unordered_set<std::string>& res);
  void getRelationWith(ClassBranch_t* class_branch, std::map<std::string, int>& properties, std::vector<int>& depths, std::vector<std::string>& res, int depth);
  void dataGetRelatedWith(ClassBranch_t* class_branch, const std::string& property, const std::string& _class, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake);
  void objectGetRelatedWith(ClassBranch_t* class_branch, const std::string& property, const std::string& _class, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake);
  void getOn(ClassBranch_t* class_branch, std::unordered_set<uint32_t>& object_properties, std::unordered_set<uint32_t>& data_properties, std::unordered_set<std::string>& res, uint32_t current_depth, int& found_depth);
  void getWith(ClassBranch_t* first_class, const std::string& second_class, std::unordered_set<std::string>& res, std::unordered_set<uint32_t>& doNotTake, uint32_t current_depth, int& found_depth, int depth_prop);

  void isMyDisjoint(ClassBranch_t* me, const std::string& disjoint, std::vector<ClassBranch_t*>& vect, bool& find, bool all = true)
  {
    if(find)
      return;

    for(size_t i = 0; i < vect.size(); i++)
      if(disjoint == vect[i]->value())
      {
        me->setSteady_disjoint(vect[i]);
        if(all)
          vect[i]->disjoints_.push_back(me);
        find = true;
        break;
      }
  }

  void isMyObjectPropertiesOn(ClassBranch_t* me, const std::string& propertyOn, std::vector<ClassBranch_t*>& vect, bool& find)
  {
    if(find)
      return;

    for(size_t i = 0; i < vect.size(); i++)
      if(propertyOn == vect[i]->value())
      {
        me->setSteady_object_properties_on(vect[i]);
        find = true;
        break;
      }
  }
};

#endif /* CLASSGRAPH_H */
