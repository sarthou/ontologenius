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


  struct ClassAtom
  {
    AnonymousClassElement* class_expression; // ClassBranch or ClassExpression
    std::string var;
    IndividualBranch* individual_involved;

    ClassAtom() : class_expression(nullptr), var(""), individual_involved(nullptr) {}
  };

  struct ObjectPropertyAtom
  {
    ObjectPropertyBranch* object_property_expression; // ObjectPropertyBranch*
    std::string var1;
    std::string var2;
    IndividualBranch* individual_involved_1;
    IndividualBranch* individual_involved_2;

    ObjectPropertyAtom() : object_property_expression(nullptr), var1(""), var2(""), individual_involved_1(nullptr), individual_involved_2(nullptr) {}
  };

  struct DataPropertyAtom
  {
    DataPropertyBranch* data_property_expression; // DataPropertyBranch*
    std::string var1;
    std::string var2;
    IndividualBranch* individual_involved;
    LiteralNode* datatype_involved;

    DataPropertyAtom() : data_property_expression(nullptr), var1(""), var2(""), individual_involved(nullptr), datatype_involved(nullptr) {}
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