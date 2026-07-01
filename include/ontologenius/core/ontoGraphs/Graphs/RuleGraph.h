#ifndef ONTOLOGENIUS_RULEGRAPH_H
#define ONTOLOGENIUS_RULEGRAPH_H

#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"

namespace ontologenius {

  struct RuleAtomDescriptor_t
  {
    RuleAtomType_e type;
    RuleBuiltinType_e builtin;
    std::string resource_value;
    ClassExpressionDescriptor_t* class_expression;

    RuleAtomDescriptor_t() : type(rule_atom_unknown), builtin(builtin_unknon), class_expression(nullptr) {}
  };

  struct RuleVariableDescriptor_t
  {
    std::string name;
    size_t index;
    bool is_instanciated;
    bool datatype;
    RuleVariableDescriptor_t() : is_instanciated(false), datatype(false) {}
  };

  struct RuleDescriptor_t
  {
    std::vector<std::string> variables;
    std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>> antecedents;
    std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>> consequents;
    std::string rule_str;
    std::map<std::string, std::vector<std::string>> comments_;

    std::string toString() const
    {
      return toString(antecedents) + " -> " + toString(consequents);
    }

    static std::string builtinToString(const RuleAtomDescriptor_t& atom)
    {
      switch(atom.builtin)
      {
      case builtin_greater_than: return "greaterThan";
      case builtin_greater_than_or_equal: return "greaterThanOrEqual";
      case builtin_less_than: return "lessThan";
      case builtin_less_than_or_equal: return "lessThanOrEqual";
      case builtin_equal: return "equal";
      case builtin_not_equal: return "notEqual";
      default: return "unsupported builtin";
      }
    }

    std::string toString(const std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>>& expression) const
    {
      std::string res;
      for(const auto& atom : expression)
      {
        if(res.empty() == false)
          res += ", ";

        switch(atom.first.type)
        {
        case RuleAtomType_e::rule_atom_data:
        case RuleAtomType_e::rule_atom_object:
          res += atom.first.resource_value;
          break;
        case RuleAtomType_e::rule_atom_class:
          res += (atom.first.class_expression == nullptr) ? atom.first.resource_value : atom.first.class_expression->toString();
          break;
        case RuleAtomType_e::rule_atom_builtin:
          res += builtinToString(atom.first);
          break;
        default:
          break;
        }

        std::string variables_str;
        for(const auto& variable : atom.second)
        {
          if(variables_str.empty() == false)
            variables_str += ", ";

          variables_str += (variable.is_instanciated ? "" : "?") + variable.name;
        }
        res += "(" + variables_str + ")";
      }
      return res;
    }
  };

  class OntologyGraphs;

  class RuleGraph : public Graph<RuleBranch>
  {
  public:
    RuleGraph(OntologyGraphs* graphs);
    RuleGraph(const RuleGraph& other, OntologyGraphs* graphs);
    ~RuleGraph() override = default;

    RuleBranch* add(const RuleDescriptor_t& rule);

    void deepCopy(const RuleGraph& other);

    const std::set<std::string>& getVariables() const { return variable_names_; }

  private:
    OntologyGraphs* graphs_;

    std::set<std::string> variable_names_; // only used to rewrite the variable fields

    RuleTriplet_t createRuleAtomTriplet(RuleBranch* rule_branch, const std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>& atom, const size_t& elem_id, const bool& is_head);

    RuleArgument_t getRuleArgument(RuleBranch* rule_branch, const RuleVariableDescriptor_t& variable);
    void setVariableIndex(RuleBranch* rule_branch, RuleArgument_t& resource);

    // functions for deepcopy
    void cpyBranch(RuleBranch* old_branch, RuleBranch* new_branch);
    RuleTriplet_t createCopyRuleTriplet(const RuleTriplet_t& old_triplet, RuleBranch* new_branch);
  };

} // namespace ontologenius
#endif // ONTOLOGENIUS_RULEGRAPH_H