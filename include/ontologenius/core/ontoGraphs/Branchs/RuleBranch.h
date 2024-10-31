#ifndef ONTOLOGENIUS_RULEBRANCH_H
#define ONTOLOGENIUS_RULEBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

  struct ClassAtom_t
  {
    // the class expression is needed for the RuleChecker
    // AnonymousClassBranch* class_expression; // stores the expression (hasCamera some Camera/Component) and the name of the element __rule_1_2
    AnonymousClassElement* class_expression; // ClassBranch or ClassExpression if ano class-> create equiv class with name __rule_1_2 and add it to ano_graph with hidden = true
    ClassBranch* equivalent_class;           // ClassBranch or equiv Anonymous class (if class then directly the pointer, else the newly created hidden ClassBranch)

    std::string var;
    IndividualBranch* individual_involved;

    ClassAtom_t() : class_expression(nullptr), equivalent_class(nullptr), individual_involved(nullptr) {}
  };

  struct ObjectPropertyAtom_t
  {
    ObjectPropertyBranch* object_property_expression; // ObjectPropertyBranch*
    std::string var1;
    std::string var2;
    IndividualBranch* individual_involved_1;
    IndividualBranch* individual_involved_2;

    ObjectPropertyAtom_t() : object_property_expression(nullptr), individual_involved_1(nullptr), individual_involved_2(nullptr) {}

    std::string toString()
    {
      std::string res;

      res = object_property_expression->value() + "(";
      if(var1.empty() == false)
        res += "?" + var1;
      else if(individual_involved_1 != nullptr)
        res += individual_involved_1->value();

      res += ", ";
      if(var2.empty() == false)
        res += "?" + var2;
      else if(individual_involved_2 != nullptr)
        res += individual_involved_2->value();
      res += ")";

      return res;
    }
  };

  struct DataPropertyAtom_t
  {
    DataPropertyBranch* data_property_expression; // DataPropertyBranch*
    std::string var1;
    std::string var2;
    IndividualBranch* individual_involved;
    LiteralNode* datatype_involved;

    DataPropertyAtom_t() : data_property_expression(nullptr), individual_involved(nullptr), datatype_involved(nullptr) {}

    std::string toString()
    {
      std::string res;

      res = data_property_expression->value() + "(";
      if(var1.empty() == false)
        res += "?" + var1;
      else if(individual_involved != nullptr)
        res += individual_involved->value();

      res += ", ";
      if(var2.empty() == false)
        res += "?" + var2;
      else if(datatype_involved != nullptr)
        res += datatype_involved->value();
      res += ")";

      return res;
    }
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
    explicit RuleBranch(const std::string& value, bool hidden = false) : ValuedNode(value, hidden) {}

    RuleAtomList_t rule_antecedents_;
    RuleAtomList_t rule_consequents_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_RULEBRANCH_H