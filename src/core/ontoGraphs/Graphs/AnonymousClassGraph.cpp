#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"

#include <cstddef>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/utils/String.h"

// #define DEBUG

namespace ontologenius {

  AnonymousClassGraph::AnonymousClassGraph(ClassGraph* class_graph,
                                           ObjectPropertyGraph* object_property_graph,
                                           DataPropertyGraph* data_property_graph,
                                           IndividualGraph* individual_graph) : class_graph_(class_graph),
                                                                                object_property_graph_(object_property_graph),
                                                                                data_property_graph_(data_property_graph),
                                                                                individual_graph_(individual_graph)
  {}

  AnonymousClassGraph::AnonymousClassGraph(const AnonymousClassGraph& other,
                                           ClassGraph* class_graph,
                                           ObjectPropertyGraph* object_property_graph,
                                           DataPropertyGraph* data_property_graph,
                                           IndividualGraph* individual_graph) : class_graph_(class_graph),
                                                                                object_property_graph_(object_property_graph),
                                                                                data_property_graph_(data_property_graph),
                                                                                individual_graph_(individual_graph)

  {
    for(auto* branch : other.all_branchs_)
    {
      auto* class_branch = new AnonymousClassBranch(branch->value());
      all_branchs_.push_back(class_branch);
    }
  }

  AnonymousClassElement* AnonymousClassGraph::createElement(ExpressionMember_t* exp, AnonymousClassElement* root_node)
  {
    AnonymousClassElement* ano_element = new AnonymousClassElement(exp->toString());
    ano_element->is_complex = exp->is_complex;
    Restriction_t current_rest = exp->rest;

    if(root_node != nullptr)
      ano_element->root_node_ = root_node;
    else
      ano_element->root_node_ = ano_element;

    // ============= Node type =================
    if(exp->logical_type_ != logical_none)
    {
      ano_element->logical_type_ = exp->logical_type_;
      return ano_element;
    }
    else if(exp->oneof == true)
    {
      ano_element->oneof = true;
      return ano_element;
    }

    // ============  Property ==================
    if(!current_rest.property.empty())
    {
      if(exp->is_data_property)
      {
        ano_element->data_property_involved_ = data_property_graph_->findOrCreateBranch(current_rest.property);
        ano_element->root_node_->involves_data_property = true;
      }
      else
      {
        ano_element->object_property_involved_ = object_property_graph_->findOrCreateBranch(current_rest.property);
        ano_element->root_node_->involves_object_property = true;
      }
    }

    // ==============  Cardinality Type & Number =============
    if(current_rest.card.cardinality_type == "some")
      ano_element->card_.card_type_ = cardinality_some;
    else if(current_rest.card.cardinality_type == "only")
      ano_element->card_.card_type_ = cardinality_only;
    else if(current_rest.card.cardinality_type == "value")
      ano_element->card_.card_type_ = cardinality_value;
    else if(current_rest.card.cardinality_type == "exactly")
    {
      ano_element->card_.card_type_ = cardinality_exactly;
      ano_element->card_.card_number_ = std::stoi(current_rest.card.cardinality_number);
    }
    else if(current_rest.card.cardinality_type == "min")
    {
      ano_element->card_.card_type_ = cardinality_min;
      ano_element->card_.card_number_ = std::stoi(current_rest.card.cardinality_number);
    }
    else if(current_rest.card.cardinality_type == "max")
    {
      ano_element->card_.card_type_ = cardinality_max;
      ano_element->card_.card_number_ = std::stoi(current_rest.card.cardinality_number);
    }

    // ===============  Cardinality range  (value, some, only )  ===================
    const std::string card_range = current_rest.card.cardinality_range;
    if(!card_range.empty())
    {
      if(exp->is_data_property) // data property
      {
        if(ano_element->card_.card_type_ == cardinality_value)
          ano_element->card_.card_range_ = data_property_graph_->createLiteral(card_range);
        else
        {
          const std::string type_value = card_range.substr(card_range.find("#") + 1, -1);
          ano_element->card_.card_range_ = data_property_graph_->createLiteral(type_value + "#"); // need to add the "#"
        }
        ano_element->root_node_->involves_data_property = true;
      }
      else // object property
      {
        if(ano_element->card_.card_type_ == cardinality_value) // indiv
        {
          ano_element->individual_involved_ = individual_graph_->findOrCreateBranch(card_range);
          ano_element->root_node_->involves_individual = true;
        }
        else
        { // class
          ano_element->class_involved_ = class_graph_->findOrCreateBranch(card_range);
          ano_element->root_node_->involves_class = true;
        }
      }
    }

    // ===============  Restriction range  (min, max, exactly )===================
    const std::string rest_range = current_rest.restriction_range;
    if(!rest_range.empty())
    {
      if(isIn("http://www.w3.org/", rest_range)) // literal node for complex data restriction (ClassX Eq to data_prop some (not(literal)))
      {
        const std::string type = split(rest_range, "#").back();
        ano_element->card_.card_range_ = data_property_graph_->createLiteral(type + "#"); // need to add the "#"
      }
      else if(exp->mother != nullptr && exp->mother->oneof) // individual node for oneOf (ClassX Eq to oneOf(indiv))
      {
        ano_element->individual_involved_ = individual_graph_->findOrCreateBranch(rest_range);
        ano_element->root_node_->involves_individual = true;
      }
      else
      {
        ano_element->class_involved_ = class_graph_->findOrCreateBranch(rest_range); // class node for class only restriction (ClassX Eq to ClassY)
        ano_element->root_node_->involves_class = true;
      }
    }

    return ano_element;
  }

  AnonymousClassElement* AnonymousClassGraph::createTree(ExpressionMember_t* member_node, size_t& depth, AnonymousClassElement* root_node)
  {
    size_t local_depth = depth + 1;

    AnonymousClassElement* node = createElement(member_node, root_node);

    if(root_node == nullptr)
      root_node = node;

    for(auto* child : member_node->child_members)
    {
      size_t child_depth = depth + 1;
      node->sub_elements_.push_back(createTree(child, child_depth, root_node));
      if(child_depth > local_depth)
        local_depth = child_depth;
    }

    depth = local_depth;

    return node;
  }

  AnonymousClassBranch* AnonymousClassGraph::add(const std::string& value, AnonymousClassVectors_t& ano)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<AnonymousClassBranch>::mutex_);

    const std::string ano_name = "anonymous_" + value;
    AnonymousClassBranch* anonymous_branch = new AnonymousClassBranch(ano_name);
    ClassBranch* class_branch = class_graph_->findOrCreateBranch(value);

    anonymous_branch->class_equiv_ = class_branch;
    all_branchs_.push_back(anonymous_branch);
    class_branch->equiv_relations_ = anonymous_branch;

    for(size_t i = 0; i < ano.equivalence_trees.size(); i++)
    {
      size_t depth = 0;
      AnonymousClassElement* ano_elem = createTree(ano.equivalence_trees[i], depth);
      ano_elem->ano_name = ano_name + "_" + std::to_string(i);
      anonymous_branch->ano_elems_.push_back(ano_elem);
      anonymous_branch->depth_ = depth;

      // std::cout << " c : " << ano_elem->root_node_->involves_class << " o : " << ano_elem->root_node_->involves_object_property
      //           << " d : " << ano_elem->root_node_->involves_data_property << " i : " << ano_elem->root_node_->involves_individual << std::endl;

#ifdef DEBUG
      printTree(ano_elem, 3, true);
#endif
    }

    return anonymous_branch;
  }

  AnonymousClassBranch* AnonymousClassGraph::addHiddenRuleElem(const size_t& rule_id, const size_t& elem_id, ExpressionMember_t* ano_expression)
  {
    const std::string current_elem = std::to_string(rule_id) + "_" + std::to_string(elem_id);
    const std::string ano_name = "anonymous_rule_" + current_elem;
    const std::string hidden_class_name = "__rule_" + current_elem;

    AnonymousClassBranch* anonymous_branch = new AnonymousClassBranch(ano_name);

    ClassBranch* hidden_class_branch = class_graph_->findOrCreateBranch(hidden_class_name, true); // this ClassBranch has to be hidden
    hidden_class_branch->equiv_relations_ = anonymous_branch;
    anonymous_branch->class_equiv_ = hidden_class_branch;
    all_branchs_.push_back(anonymous_branch);

    size_t depth = 0;
    AnonymousClassElement* ano_elem = createTree(ano_expression, depth);
    ano_elem->ano_name = ano_name;
    anonymous_branch->ano_elems_.push_back(ano_elem);
    anonymous_branch->depth_ = depth;

    // std::cout << " c : " << ano_elem->root_node_->involves_class << " o : " << ano_elem->root_node_->involves_object_property
    //           << " d : " << ano_elem->root_node_->involves_data_property << " i : " << ano_elem->root_node_->involves_individual << std::endl;

    // std::cout << "created ano : " << anonymous_branch->value() << " and hidden class : " << hidden_class_branch->value() << std::endl;
    return anonymous_branch;
  }

  void AnonymousClassGraph::printTree(AnonymousClassElement* ano_elem, size_t level, bool root) const
  {
    const std::string space(level * 4, ' ');
    std::string tmp;

    if(root)
      std::cout << space;

    if(ano_elem->logical_type_ == LogicalNodeType_e::logical_and)
      tmp += "and";
    else if(ano_elem->logical_type_ == LogicalNodeType_e::logical_or)
      tmp += "or";
    else if(ano_elem->logical_type_ == LogicalNodeType_e::logical_not)
      tmp += "not";
    else if(ano_elem->oneof)
      tmp += "oneOf";
    else if(ano_elem->object_property_involved_ != nullptr)
    {
      tmp += ano_elem->object_property_involved_->value();
      tmp += " " + toString(ano_elem->card_.card_type_);

      if(ano_elem->card_.card_type_ == ontologenius::CardType_e::cardinality_value)
        tmp += " " + ano_elem->individual_involved_->value();
      else
      {
        if(ano_elem->card_.card_number_ != 0)
          tmp += " " + std::to_string(ano_elem->card_.card_number_);
        if(ano_elem->class_involved_ != nullptr)
          tmp += " " + ano_elem->class_involved_->value();
      }
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      tmp += ano_elem->data_property_involved_->value();
      tmp += " " + toString(ano_elem->card_.card_type_);
      if(ano_elem->card_.card_number_ != 0)
        tmp += " " + std::to_string(ano_elem->card_.card_number_);
      if(ano_elem->card_.card_range_ != nullptr)
        tmp += " " + ano_elem->card_.card_range_->value();
    }
    else
    {
      if(ano_elem->class_involved_ != nullptr)
        tmp += ano_elem->class_involved_->value();
      else if(ano_elem->individual_involved_ != nullptr)
        tmp += ano_elem->individual_involved_->value();
      else if(ano_elem->card_.card_range_ != nullptr)
        tmp += ano_elem->card_.card_range_->type_;
    }

    std::cout << tmp << std::endl;

    for(auto* sub_elem : ano_elem->sub_elements_)
    {
      for(int i = 0; i < int(level); i++)
        std::cout << "│   ";
      if(sub_elem == ano_elem->sub_elements_.back())
        std::cout << "└── ";
      else
        std::cout << "├── ";
      printTree(sub_elem, level + 1, false);
    }
  }

  std::string AnonymousClassGraph::toString(CardType_e value) const
  {
    switch(value)
    {
    case CardType_e::cardinality_some:
      return "some";
    case CardType_e::cardinality_only:
      return "only";
    case CardType_e::cardinality_min:
      return "min";
    case CardType_e::cardinality_max:
      return "max";
    case CardType_e::cardinality_exactly:
      return "exactly";
    case CardType_e::cardinality_value:
      return "value";
    case CardType_e::cardinality_error:
      return "error";
    default:
      return "";
    }
  }

} // namespace ontologenius