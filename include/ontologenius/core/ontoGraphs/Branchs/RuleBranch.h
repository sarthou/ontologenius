#ifndef ONTOLOGENIUS_RULEBRANCH_H
#define ONTOLOGENIUS_RULEBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

  // one rule has antecedent and consequent
  // antecedent can contain, classes, individuals, obj/dat prop, swrl buitins, sameAs, DifferentFrom, and class expressions
  // 3 flags selon les modifications
  // maitien d'une liste de classes non nommés pour les ano classes dans les antecedents

  class RuleClassElement
  {
  public:
    RuleClassElement() : logical_type_(LogicalNodeType_e::logical_none), oneof(false), is_complex(false),
                         class_involved_(nullptr), object_property_involved_(nullptr),
                         data_property_involved_(nullptr), individual_involved_(nullptr) {}

    LogicalNodeType_e logical_type_;
    bool oneof; // true = OneOf element
    bool is_complex;

    // pointers to the concepts used in the equivalence relation
    ClassBranch* class_involved_;
    ObjectPropertyBranch* object_property_involved_;
    DataPropertyBranch* data_property_involved_;
    IndividualBranch* individual_involved_;

    CardinalityElement_t card_;

    std::vector<RuleClassElement*> sub_elements_;
    std::string rule_id;
  };

  struct ClassAtom
  {
    RuleClassElement* class_expression; // ClassBranch or ClassExpression
    std::string var;
    IndividualBranch* individual_involved; // either var or individual_involved is filled
  };

  struct ObjectPropertyAtom
  {
    ObjectPropertyBranch* object_property_expression; // ObjectPropertyBranch*
    std::string var1;
    std::string var2;
    IndividualBranch* individual_involved_1;
    IndividualBranch* individual_involved_2;
  };

  struct DataPropertyAtom
  {
    DataPropertyBranch* data_property_expression; // DataPropertyBranch*
    std::string var1;
    std::string var2; // stored as type#value (e.g boolean#true)
    IndividualBranch* individual_involved;
    LiteralNode* datatype_involved;
  };

  struct BuiltinAtom
  {
    std::string builtin_type;
    std::vector<std::string> vars;
  };

  struct RuleAtomList
  {
    std::vector<ClassAtom*> class_atoms_;
    std::vector<ObjectPropertyAtom*> object_atoms_;
    std::vector<DataPropertyAtom*> data_atoms_;
    std::vector<BuiltinAtom*> builtin_atoms_;
  };

  class RuleBranch : public ValuedNode
  {
  public:
    explicit RuleBranch(const std::string& value) : ValuedNode(value) {}

    RuleAtomList* rule_antecedents_;
    RuleAtomList* rule_consequents_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_RULEBRANCH_H