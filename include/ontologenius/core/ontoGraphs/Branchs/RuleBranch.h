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

  struct RuleResource_t
  {
    // empty constructor
    RuleResource_t() : variable_id(-1),
                       is_variable(false),
                       indiv_value(nullptr),
                       datatype_value(nullptr)
    {}
    // variable constructor
    RuleResource_t(const std::string& name_value) : name(name_value),
                                                    is_variable(true),
                                                    indiv_value(nullptr),
                                                    datatype_value(nullptr)
    {}
    // instantiated individual variable constructor
    RuleResource_t(IndividualBranch* indiv) : name(indiv->value()),
                                              is_variable(false),
                                              indiv_value(indiv),
                                              datatype_value(nullptr)
    {}

    // instantiated literal variable constructor
    RuleResource_t(LiteralNode* literal) : name(literal->toString()),
                                           is_variable(false),
                                           indiv_value(nullptr),
                                           datatype_value(literal)
    {}

    std::string name;
    int64_t variable_id;
    bool is_variable;
    IndividualBranch* indiv_value;
    LiteralNode* datatype_value;
  };

  enum AtomType_e
  {
    default_atom,
    class_atom,
    object_atom,
    data_atom,
    builtin_atom
  };

  struct RuleTriplet_t
  {
    // empty constructor
    RuleTriplet_t() : atom_type_(default_atom), class_predicate(nullptr), class_element(nullptr), object_predicate(nullptr), data_predicate(nullptr)
    {}

    // simple class triplet
    RuleTriplet_t(ClassBranch* class_branch,
                  RuleResource_t& resource) : atom_type_(class_atom),
                                              subject(resource),
                                              class_predicate(class_branch),
                                              class_element(nullptr),
                                              object_predicate(nullptr),
                                              data_predicate(nullptr)
    {}

    // complex class triplet
    RuleTriplet_t(ClassBranch* class_branch,
                  AnonymousClassElement* ano_expression,
                  RuleResource_t& resource) : atom_type_(class_atom),
                                              subject(resource),
                                              class_predicate(class_branch),
                                              class_element(ano_expression),
                                              object_predicate(nullptr),
                                              data_predicate(nullptr)
    {}

    // object triplet
    RuleTriplet_t(RuleResource_t& resource_from,
                  ObjectPropertyBranch* property,
                  RuleResource_t& resource_on) : atom_type_(object_atom),
                                                 subject(resource_from),
                                                 class_predicate(nullptr),
                                                 class_element(nullptr),
                                                 object_predicate(property),
                                                 data_predicate(nullptr),
                                                 object(resource_on)
    {}

    // data triplet
    RuleTriplet_t(RuleResource_t& resource_from,
                  DataPropertyBranch* property,
                  RuleResource_t& resource_on) : atom_type_(data_atom),
                                                 subject(resource_from),
                                                 class_predicate(nullptr),
                                                 class_element(nullptr),
                                                 object_predicate(nullptr),
                                                 data_predicate(property),
                                                 object(resource_on)
    {}

    // builtin triplet
    RuleTriplet_t(RuleResource_t& resource_from,
                  Builtin_t& builtin,
                  RuleResource_t& resource_on) : atom_type_(builtin_atom),
                                                 subject(resource_from),
                                                 class_predicate(nullptr),
                                                 class_element(nullptr),
                                                 object_predicate(nullptr),
                                                 data_predicate(nullptr),
                                                 builtin(builtin),
                                                 object(resource_on)
    {}

    AtomType_e atom_type_;

    RuleResource_t subject;                 // can be variable or not (?c is, pr2 isn't)
    ClassBranch* class_predicate;           // set only if class atom
    AnonymousClassElement* class_element;   // used to store the anonymous class if the class expression is complex
    ObjectPropertyBranch* object_predicate; // set only if object atom
    DataPropertyBranch* data_predicate;     // set only if data atom
    Builtin_t builtin;                      // used only for builtin atoms
    RuleResource_t object;                  // can be variable or not (realsense i not), uninstantiated if class atom since it doesnt have another variable

    std::string toString() const
    {
      std::string res;

      switch(atom_type_)
      {
      case class_atom:
        res = class_predicate->value();
        if(subject.is_variable == true)
          res += "(?" + subject.name + ")";
        else
          res += "(" + subject.name + ")";
        break;
      case object_atom:
        res = object_predicate->value();
        if(subject.is_variable == true)
          res += "(?" + subject.name;
        else
          res += "(" + subject.name;

        if(object.is_variable == true)
          res += ", ?" + object.name + ")";
        else
          res += ", " + object.name + ")";
        break;
      case data_atom:
        res = data_predicate->value();

        if(subject.is_variable == true)
          res += "(?" + subject.name;
        else
          res += "(" + subject.name;

        if(object.is_variable == true)
          res += ", ?" + object.name + ")";
        else
          res += ", " + object.name + ")";
        break;
      case builtin_atom:
        /* code */
        break;

      default:
        break;
      }
      return res;
    }
  };

  struct Variable_t
  {
    Variable_t() : is_instantiated(false), is_datavalue(false), is_builtin_value(false), var_index(-1) {}

    std::string var_name;
    bool is_instantiated; // for indiv
    bool is_datavalue; // for literal
    bool is_builtin_value; // for builtin data
    int64_t var_index;

    std::string toString() const { return var_name; }
  };

  class RuleBranch : public ValuedNode,
                     public InferenceRuleNode
  {
  public:
    explicit RuleBranch(const std::string& value,
                        const std::string& rule,
                        bool hidden = false) : ValuedNode(value, hidden),
                                               InferenceRuleNode(rule),
                                               involves_class(false),
                                               involves_object_property(false),
                                               involves_data_property(false)
    {}

    bool involves_class;
    bool involves_object_property;
    bool involves_data_property;

    std::vector<RuleTriplet_t> rule_body_;
    std::vector<RuleTriplet_t> rule_head_;

    // mapping between variables and creation of the RuleResource_t elements
    std::unordered_map<std::string, int64_t> variables_; // mapping between var names and index
    std::vector<std::string> to_variables_;              // mapping between index and var name

    std::vector<size_t> atom_initial_order_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_RULEBRANCH_H