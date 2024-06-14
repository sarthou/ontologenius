#ifndef ONTOLOGENIUS_INDIVIDUALBRANCH_H
#define ONTOLOGENIUS_INDIVIDUALBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

  // Classes predefinition
  class IndividualBranch;

  using IndividualElement = SingleElement<IndividualBranch*>;
  using IndivObjectRelationElement = PairElement<ObjectPropertyBranch*, IndividualBranch*>;
  using IndivDataRelationElement = PairElement<DataPropertyBranch*, LiteralNode*>;

  class IndividualBranch : public ValuedNode
  {
  public:
    RelationsWithInductions<ClassElement> is_a_;
    RelationsWithInductions<IndivObjectRelationElement> object_relations_;
    RelationsWithInductions<IndivDataRelationElement> data_relations_;
    RelationsWithInductions<IndividualElement> same_as_;
    std::vector<IndividualElement> distinct_;

    IndividualBranch(const std::string& value = "") : ValuedNode(value) {}

    int objectRelationExists(ObjectPropertyBranch* property, IndividualBranch* individual);
    int dataRelationExists(DataPropertyBranch* property, LiteralNode* data);

    bool hasUpdatedObjectRelation();
    bool hasUpdatedDataRelation();
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALBRANCH_H
