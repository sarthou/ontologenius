#ifndef ONTOLOGENIUS_CLASSBRANCH_H
#define ONTOLOGENIUS_CLASSBRANCH_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/Branch.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/Data.h"

namespace ontologenius {

// Classes predefinition
class ObjectPropertyBranch_t;
class DataPropertyBranch_t;
class IndividualBranch_t;
class ClassBranch_t;

typedef Single_t<IndividualBranch_t*> IndividualElement_t;
typedef Single_t<ClassBranch_t*> ClassElement_t;
typedef Pair_t<ObjectPropertyBranch_t*, ClassBranch_t*> ClassObjectRelationElement_t;
typedef Pair_t<DataPropertyBranch_t*, data_t> ClassDataRelationElement_t;

template <typename T>
class ClassBranchData_t
{
public:
  std::vector<IndividualElement_t> individual_childs_;

  std::vector<T*> disjoints_;

  std::vector<Pair_t<ObjectPropertyBranch_t*, T*>> object_relations_;
  std::vector<ClassDataRelationElement_t> data_relations_;
};

class ClassSteady_t : public BranchSteady_t<ClassBranch_t>, public ClassBranchData_t<ClassBranch_t>
{
public:
  ClassSteady_t() {}
};

class ClassBranch_t : public Branch_t<ClassBranch_t>, public ClassBranchData_t<ClassBranch_t>
{
public:
  ClassSteady_t steady_;

  ClassBranch_t(std::string value = "") : Branch_t(value) {};

  void setFullSteady();
  void setSteady_disjoint(ClassBranch_t* disjoint);
  void setSteady_child(const ClassElement_t& child);
  void setSteady_individual_child(const IndividualElement_t& child);
  void setSteady_mother(const ClassElement_t& mother);
  void setSteady_dictionary(std::string lang, std::string word);
  void setSteady_muted_dictionary(std::string lang, std::string word);
  void setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
  void setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary);

  void setSteady_objectRelation(const ClassObjectRelationElement_t& object_relation);
  void setSteady_dataRelation(const ClassDataRelationElement_t& data_relation);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSBRANCH_H
