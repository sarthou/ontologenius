#ifndef ONTOLOGENIUS_INDIVIDUALBRANCH_H
#define ONTOLOGENIUS_INDIVIDUALBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

namespace ontologenius {

// Classes predefinition
class IndividualBranch_t;

typedef Triplet_t<IndividualBranch_t, ObjectPropertyBranch_t, IndividualBranch_t> ObjectRelationTriplet_t;
typedef Triplets<IndividualBranch_t, ObjectPropertyBranch_t, IndividualBranch_t> ObjectRelationTriplets;

typedef Single_t<IndividualBranch_t*> IndividualElement_t;
typedef Pair_t<ObjectPropertyBranch_t*, IndividualBranch_t*> IndivObjectRelationElement_t;
typedef Pair_t<DataPropertyBranch_t*, LiteralNode*> IndivDataRelationElement_t;

class IndividualBranch_t : public ValuedNode
{
  // as classes member, object property members, data property members can allow to induce class equiv, higher level class to pack everything
  // instead of having std::vector<Triplets> object_properties_has_induced_; std::vector<Triplets> data_properties_has_induced_; std::vector<Triplets> is_a_has_induced;
  // so we design a template class which can take all of these members to induce relations like std::vector<Induced_relations>

  // reasons of equivalence inducing : 
  // - obj prop card indiv
  // - data prop card indiv
public:
  std::vector<ClassElement_t> is_a_;
  std::vector<IndivObjectRelationElement_t> object_relations_;
  std::vector<ObjectRelationTriplets> object_properties_has_induced_;
  std::vector<IndivDataRelationElement_t> data_relations_;
  std::vector<IndividualElement_t> same_as_;
  std::vector<IndividualElement_t> distinct_;

  IndividualBranch_t(const std::string& value = "") : ValuedNode(value) {}

  void setSteady_dictionary(const std::string& lang, const std::string& word);
  void setSteady_muted_dictionary(const std::string& lang, const std::string& word);
  void setSteady_dictionary(const std::map<std::string, std::vector<std::string>>& dictionary);
  void setSteady_muted_dictionary(const std::map<std::string, std::vector<std::string>>& dictionary);

  int objectPropertyExist(ObjectPropertyBranch_t* property, IndividualBranch_t* individual);
  int dataPropertyExist(DataPropertyBranch_t* property, LiteralNode* data);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALBRANCH_H
