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

  struct ClassAtom_t
  {
    AnonymousClassElement* class_expression; // ClassBranch or ClassExpression
    std::string var;
    IndividualBranch* individual_involved;

    ClassAtom_t() : class_expression(nullptr), individual_involved(nullptr) {}
  };

  struct ObjectPropertyAtom_t
  {
    ObjectPropertyBranch* object_property_expression; // ObjectPropertyBranch*
    std::string var1;
    std::string var2;
    IndividualBranch* individual_involved_1;
    IndividualBranch* individual_involved_2;

    ObjectPropertyAtom_t() : object_property_expression(nullptr), individual_involved_1(nullptr), individual_involved_2(nullptr) {}
  };

  struct DataPropertyAtom_t
  {
    DataPropertyBranch* data_property_expression; // DataPropertyBranch*
    std::string var1;
    std::string var2;
    IndividualBranch* individual_involved;
    LiteralNode* datatype_involved;

    DataPropertyAtom_t() : data_property_expression(nullptr), individual_involved(nullptr), datatype_involved(nullptr) {}
  };

  struct BuiltinAtom_t
  {
    std::string builtin_type;
    std::vector<std::string> vars;
  };

  struct RuleAtomList_t
  {
    std::vector<ClassAtom_t*> class_atoms_;
    std::vector<ObjectPropertyAtom_t*> object_atoms_;
    std::vector<DataPropertyAtom_t*> data_atoms_;
    std::vector<BuiltinAtom_t*> builtin_atoms_;
  };

  class RuleBranch : public ValuedNode
  {
  public:
    explicit RuleBranch(const std::string& value) : ValuedNode(value) {}

    RuleAtomList_t rule_antecedents_;
    RuleAtomList_t rule_consequents_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_RULEBRANCH_H