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

  struct Rule_t // temporary structure to store rules
  {
    std::vector<std::string> variables;
    std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>> antecedents;
    std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>> consequents;
    std::string rule_str;
    std::string rule_comment;

    std::vector<std::pair<int64_t, int64_t>> atom_indexes_;

    std::string toStringRule()
    {
      return toString(antecedents) + " -> " + toString(consequents);
    }

    std::string toString(std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>>& expression) const
    {
      std::string res;

      std::size_t len_head = expression.size();
      for(size_t element_index = 0; element_index < len_head; element_index++)
      {
        // get the Atom expression
        if(expression[element_index].first != nullptr)
        {
          if(res.empty() == false)
            res += ", ";

          if(expression[element_index].first->logical_type_ != logical_none || expression[element_index].first->is_complex)
            res += "(" + expression[element_index].first->toString() + ")";
          else
            res += expression[element_index].first->toString();

          res += "(";
          // get the associated variables
          std::size_t len_var = expression[element_index].second.size();
          for(size_t var_index = 0; var_index < len_var; var_index++)
          {
            auto variable = expression[element_index].second[var_index];
            std::string var_name = variable.var_name;
            if(variable.is_datavalue)
            {
              std::size_t pos = var_name.find('#');
              if(pos != std::string::npos) // datarange value
                res += var_name.substr(pos + 1);
            }
            else
            {
              if(variable.is_instantiated) // individual name
                res += var_name;
              else // variable so we add the ?
                res += "?" + var_name;
            }
            if(var_index < len_var - 1)
              res += ", ";
          }
          res += ")";
        }
      }
      return res;
    }
  };

  // for friend
  class ObjectPropertyGraph;
  class DataPropertyGraph;
  class IndividualGraph;
  class ClassGraph;
  class AnonymousClassGraph;

  class RuleChecker;
  class RuleOwlWriter;

  class RuleGraph : public Graph<RuleBranch>
  {
    friend ObjectPropertyGraph;
    friend DataPropertyGraph;
    friend IndividualGraph;
    friend ClassGraph;
    friend AnonymousClassGraph;
    friend RuleOwlWriter;

    friend RuleChecker;

  public:
    RuleGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph,
              DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph);
    RuleGraph(const RuleGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph,
              DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph);
    ~RuleGraph() override = default;

    RuleBranch* add(const std::size_t& rule_id, Rule_t& rule);

    RuleTriplet_t createRuleAtomTriplet(RuleBranch* rule_branch, const std::pair<ontologenius::ExpressionMember_t*, std::vector<ontologenius::Variable_t>>& rule_element, const size_t& rule_id, const size_t& elem_id, const bool& is_head);
    RuleTriplet_t createClassTriplet(RuleBranch* rule_branch, ExpressionMember_t* class_member, const Variable_t& variable, const size_t& rule_id, const size_t& elem_id);
    RuleTriplet_t createObjectPropertyTriplet(RuleBranch* rule_branch, ExpressionMember_t* property_member, const std::vector<Variable_t>& variable);
    RuleTriplet_t createDataPropertyTriplet(RuleBranch* rule_branch, ExpressionMember_t* property_member, const std::vector<Variable_t>& variable);
    RuleTriplet_t createBuiltinTriplet(RuleBranch* rule_branch, ExpressionMember_t* property_member, const std::vector<Variable_t>& variable);

    RuleResource_t getRuleResource(RuleBranch* rule_branch, const Variable_t& variable);
    void setVariableIndex(RuleBranch* rule_branch, RuleResource_t& resource);

  private:
    ClassGraph* class_graph_;
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;
    IndividualGraph* individual_graph_;
    AnonymousClassGraph* anonymous_graph_;

    std::set<std::string> variable_names_; // only used to rewrite the variable fields
  };

} // namespace ontologenius
#endif // ONTOLOGENIUS_RULEGRAPH_H