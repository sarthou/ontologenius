#ifndef ONTOLOGENIUS_RULEGRAPH_H
#define ONTOLOGENIUS_RULEGRAPH_H

#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"

namespace ontologenius {

  struct Variable_t
  {
    std::string var_name;
    bool is_instantiated;
    bool is_datavalue;

    std::string toString() { return var_name; }
  };

  struct Rule_t
  {
    std::vector<std::string> variables;
    std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>> antecedents;
    std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>> consequents;
    std::string rule_str;
    std::string rule_comment;

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

  class RuleGraph : public Graph<RuleBranch>
  {
    friend ObjectPropertyGraph;
    friend DataPropertyGraph;
    friend IndividualGraph;
    friend ClassGraph;
    friend AnonymousClassGraph;

    friend AnonymousClassChecker;

  public:
    RuleGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph,
              DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph);
    RuleGraph(const RuleGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph,
              DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph);
    ~RuleGraph() {}

    // AnonymousClassElement_t* createElement(ExpressionMember_t* exp_leaf);
    // void update(ExpressionMember_t* exp, AnonymousClassElement_t* ano_class);
    // AnonymousClassElement_t* createTree(ExpressionMember_t* member_node, size_t& depth);
    RuleBranch* add(const std::size_t& value, Rule_t& rule);
    std::string getVariable(Rule_t& rule);
    // AnonymousClassElement_t* resolveTree(AnonymousClassElement_t* elem,  bool prev_and);

    // void printTree(AnonymousClassElement_t* ano_elem, size_t level, bool root);
    // std::string toString(CardType_t value);

  private:
    ClassGraph* class_graph_;
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;
    IndividualGraph* individual_graph_;
    AnonymousClassGraph* anonymous_graph_;
  };

} // namespace ontologenius
#endif // ONTOLOGENIUS_RULEGRAPH_H