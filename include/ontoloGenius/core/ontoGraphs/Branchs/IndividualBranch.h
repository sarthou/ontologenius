#ifndef ONTOLOGENIUS_INDIVIDUALBRANCH_H
#define ONTOLOGENIUS_INDIVIDUALBRANCH_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/ValuedNode.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

namespace ontologenius {

// Classes predefinition
class Triplet;
class IndividualBranch_t;

typedef Single_t<IndividualBranch_t*> IndividualElement_t;
typedef Pair_t<ObjectPropertyBranch_t*, IndividualBranch_t*> IndivObjectRelationElement_t;
typedef Pair_t<DataPropertyBranch_t*, data_t> IndivDataRelationElement_t;

template <typename T>
class IndividualBranchData_t
{
public:
  std::vector<ClassElement_t> is_a_;

  std::vector<Pair_t<ObjectPropertyBranch_t*, T*>> object_relations_;
  std::vector<Triplet> object_properties_has_induced_;

  std::vector<IndivDataRelationElement_t> data_relations_;

  std::vector<T*> same_as_;
  std::vector<T*> distinct_;
};

class IndividualSteady_t : public ValuedNodeData, public IndividualBranchData_t<IndividualBranch_t>
{
public:
  IndividualSteady_t() {}
};

class IndividualBranch_t : public ValuedNode, public IndividualBranchData_t<IndividualBranch_t>
{
public:
  bool mark;
  IndividualSteady_t steady_;

  IndividualBranch_t(std::string value = "") : ValuedNode(value) {mark = false; }

  void setFullSteady();
  void setSteady_is_a(const ClassElement_t& is_a);

  void setSteady_objectRelation(const IndivObjectRelationElement_t& object_relation);
  void setSteady_dataRelation(const IndivDataRelationElement_t& data_relation);

  void setSteady_same_as(IndividualBranch_t* same_as);
  void setSteady_distinct(IndividualBranch_t* distinct);
  void setSteady_dictionary(std::string lang, std::string word);
  void setSteady_muted_dictionary(std::string lang, std::string word);
  void setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
  void setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary);

  int ObjectPropertyExistSteady(ObjectPropertyBranch_t* property, IndividualBranch_t* individual);
  int ObjectPropertyExist(ObjectPropertyBranch_t* property, IndividualBranch_t* individual);
};

class Triplet
{
public:
  void push(IndividualBranch_t* from,
            ObjectPropertyBranch_t* prop,
            IndividualBranch_t* on)
  {
    from_.push_back(from);
    prop_.push_back(prop);
    on_.push_back(on);
  }
  bool exist(IndividualBranch_t* from,
            ObjectPropertyBranch_t* prop,
            IndividualBranch_t* on)
  {
    for(auto from_i : from_)
      if(from_i == from)
        for(auto prop_i : prop_)
          if(prop_i == prop)
            for(auto on_i : on_)
              if(on_i == on)
                return true;
    return false;
  }
  std::vector<IndividualBranch_t*> from_;
  std::vector<ObjectPropertyBranch_t*> prop_;
  std::vector<IndividualBranch_t*> on_;

};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALBRANCH_H
