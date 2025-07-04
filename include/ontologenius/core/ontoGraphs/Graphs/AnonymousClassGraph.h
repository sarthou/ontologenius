#ifndef ONTOLOGENIUS_ANOCLASSGRAPH_H
#define ONTOLOGENIUS_ANOCLASSGRAPH_H

#include <cstddef>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"

namespace ontologenius {

  struct Cardinality_t
  {
    std::string cardinality_type;
    std::string cardinality_number;
    std::string cardinality_range;

    std::string toString() const
    {
      std::string res;
      if(cardinality_type.empty() == false)
        res += " " + cardinality_type + " ";
      if(cardinality_number.empty() == false)
        res += cardinality_number + " ";
      if(cardinality_range.empty() == false)
        res += cardinality_range;
      return res;
    }

    std::vector<std::string> toVector() const
    {
      return {cardinality_range, cardinality_number, cardinality_type};
    }
  };

  struct Restriction_t
  {
    Cardinality_t card;
    std::string property;
    std::string restriction_range;

    std::string toString() const
    {
      std::string res;

      if(!property.empty())
        res += property;

      res += card.toString();

      if(!restriction_range.empty())
        res += restriction_range;

      return res;
    }

    std::vector<std::string> toVector() const
    {
      std::vector<std::string> result;
      if(property.empty() == false)
        result.push_back(property);
      if(card.cardinality_type.empty() == false)
        result.push_back(card.cardinality_type);
      if(card.cardinality_number.empty() == false)
        result.push_back(card.cardinality_number);
      if(card.cardinality_range.empty() == false)
        result.push_back(card.cardinality_range);
      if(restriction_range.empty() == false)
        result.push_back(restriction_range);

      return result;
    }
  };

  struct ExpressionMember_t
  {
    LogicalNodeType_e logical_type_;
    bool oneof;
    bool is_complex;
    bool is_data_property;

    // for SWRL rule use only
    Builtin_t builtin_;

    Restriction_t rest; // Restriction (e.g hasComponent some Camera)
    std::vector<ExpressionMember_t*> child_members;
    ExpressionMember_t* mother;
    std::string ano_name;

    ExpressionMember_t() : logical_type_(logical_none), oneof(false),
                           is_complex(false), is_data_property(false),
                           mother(nullptr) {}

    std::string toString()
    {
      std::string str_equivalence;

      if(builtin_.builtin_type_ != builtin_none)
        str_equivalence = builtin_.builtinToString();
      else if(child_members.empty())
        str_equivalence = rest.toString();
      else if(logical_type_ == logical_not)
        str_equivalence = "not (" + child_members.front()->toString() + ")";
      else
      {
        std::string inner;
        for(auto* child : child_members)
        {
          if(inner.empty() == false)
          {
            if(logical_type_ == logical_and)
              inner += " and ";
            else if(logical_type_ == logical_or)
              inner += " or ";
            else if(oneof == true)
              inner += ", ";
          }
          if(oneof == true)
            inner += child->toString();
          else
            inner += "(" + child->toString() + ")";
        }

        if(oneof == true)
          str_equivalence = "oneOf (" + inner + ")";
        else if(is_complex)
          str_equivalence = rest.toString() + inner;
        else
          str_equivalence = inner;
      }

      return str_equivalence;
    }
  };

  struct AnonymousClassVectors_t
  {
    std::string class_equiv;
    std::vector<ExpressionMember_t*> equivalence_trees;
    std::vector<std::string> str_equivalences;
  };

  // for friend
  class ObjectPropertyGraph;
  class DataPropertyGraph;
  class IndividualGraph;
  class ClassGraph;

  class AnonymousClassChecker;

  class AnonymousClassGraph : public Graph<AnonymousClassBranch>
  {
    friend ObjectPropertyGraph;
    friend DataPropertyGraph;
    friend IndividualGraph;
    friend ClassGraph;

    friend AnonymousClassChecker;

  public:
    AnonymousClassGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph,
                        DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
    AnonymousClassGraph(const AnonymousClassGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph,
                        DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
    ~AnonymousClassGraph() override = default;

    AnonymousClassElement* createElement(ExpressionMember_t* exp_leaf, AnonymousClassElement* root_node = nullptr);
    AnonymousClassElement* createTree(ExpressionMember_t* member_node, size_t& depth, AnonymousClassElement* root_node = nullptr);
    AnonymousClassBranch* add(const std::string& value, AnonymousClassVectors_t& ano_class);
    AnonymousClassBranch* addHiddenRuleElem(const size_t& rule_id, const size_t& elem_id, ExpressionMember_t* ano_expression); // for rule usage
    AnonymousClassElement* resolveTree(AnonymousClassElement* elem, bool prev_and);

    void printTree(AnonymousClassElement* ano_elem, size_t level, bool root) const;
    std::string toString(CardType_e value) const;

  private:
    ClassGraph* class_graph_;
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;
    IndividualGraph* individual_graph_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOCLASSGRAPH_H