#ifndef ONTOLOGENIUS_INDIVIDUALBRANCH_H
#define ONTOLOGENIUS_INDIVIDUALBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h"

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

namespace ontologenius {

// Classes predefinition
class IndividualBranch_t;

typedef Single_t<IndividualBranch_t*> IndividualElement_t;
typedef Pair_t<ObjectPropertyBranch_t*, IndividualBranch_t*> IndivObjectRelationElement_t;
typedef Pair_t<DataPropertyBranch_t*, LiteralNode*> IndivDataRelationElement_t;

class IndividualBranch_t : public ValuedNode
{
public:
  RelationsWithInductions<ClassElement_t> is_a_;
  RelationsWithInductions<IndivObjectRelationElement_t> object_relations_;
  RelationsWithInductions<IndivDataRelationElement_t> data_relations_;
  RelationsWithInductions<IndividualElement_t> same_as_;
  std::vector<IndividualElement_t> distinct_;

  IndividualBranch_t(const std::string& value = "") : ValuedNode(value) {}

  int objectRelationExists(ObjectPropertyBranch_t* property, IndividualBranch_t* individual);
  int dataRelationExists(DataPropertyBranch_t* property, LiteralNode* data);

  bool hasUpdatedObjectRelation();
  bool hasUpdatedDataRelation();
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALBRANCH_H
