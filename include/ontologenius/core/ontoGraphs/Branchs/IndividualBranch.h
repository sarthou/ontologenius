#ifndef ONTOLOGENIUS_INDIVIDUALBRANCH_H
#define ONTOLOGENIUS_INDIVIDUALBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

namespace ontologenius {

// Classes predefinition
class Triplet;
class IndividualBranch_t;

typedef Single_t<IndividualBranch_t*> IndividualElement_t;
typedef Pair_t<ObjectPropertyBranch_t*, IndividualBranch_t*> IndivObjectRelationElement_t;
typedef Pair_t<DataPropertyBranch_t*, data_t> IndivDataRelationElement_t;

class IndividualBranch_t : public ValuedNode
{
public:
  bool mark;

  std::vector<ClassElement_t> is_a_;
  std::vector<IndivObjectRelationElement_t> object_relations_;
  std::vector<Triplet> object_properties_has_induced_;
  std::vector<IndivDataRelationElement_t> data_relations_;
  std::vector<IndividualElement_t> same_as_;
  std::vector<IndividualElement_t> distinct_;

  IndividualBranch_t(const std::string& value = "") : ValuedNode(value) {mark = false; }

  void setSteady_dictionary(const std::string& lang, const std::string& word);
  void setSteady_muted_dictionary(const std::string& lang, const std::string& word);
  void setSteady_dictionary(const std::map<std::string, std::vector<std::string>>& dictionary);
  void setSteady_muted_dictionary(const std::map<std::string, std::vector<std::string>>& dictionary);

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
    for(size_t i = 0; i < from_.size(); i++)
      if((from_[i] != from) && (prop_[i] != prop) && (on_[i] == on))
        return true;
  
    return false;
  }
  std::vector<IndividualBranch_t*> from_;
  std::vector<ObjectPropertyBranch_t*> prop_;
  std::vector<IndividualBranch_t*> on_;

};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALBRANCH_H
