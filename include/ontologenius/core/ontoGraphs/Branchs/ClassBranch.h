#ifndef ONTOLOGENIUS_CLASSBRANCH_H
#define ONTOLOGENIUS_CLASSBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"

namespace ontologenius {

  // Classes predefinition
  class ObjectPropertyBranch;
  class DataPropertyBranch;
  class IndividualBranch;
  class ClassBranch;
  class AnonymousClassBranch;

  using IndividualElement = SingleElement<IndividualBranch*>;
  using ClassElement = SingleElement<ClassBranch*>;
  using ClassObjectRelationElement = PairElement<ObjectPropertyBranch*, ClassBranch*>;
  using ClassDataRelationElement = PairElement<DataPropertyBranch*, LiteralNode*>;

  class ClassBranch : public Branch<ClassBranch>
  {
  public:
    std::vector<IndividualElement> individual_childs_;
    std::vector<ClassObjectRelationElement> object_relations_;
    std::vector<ClassDataRelationElement> data_relations_;
    AnonymousClassBranch* equiv_relations_;

    ClassBranch(const std::string& value = "") : Branch(value), equiv_relations_(nullptr){};
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSBRANCH_H
