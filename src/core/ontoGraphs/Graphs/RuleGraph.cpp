#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

#include <algorithm>
#include <iostream>

#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/utils/String.h"

// #define DEBUG

namespace ontologenius {

  RuleGraph::RuleGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph,
                       IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph)
  {
    class_graph_ = class_graph;
    object_property_graph_ = object_property_graph;
    data_property_graph_ = data_property_graph;
    individual_graph_ = individual_graph;
    anonymous_graph_ = anonymous_graph;
  }

  RuleGraph::RuleGraph(const RuleGraph& other, ClassGraph* class_graph,
                       ObjectPropertyGraph* object_property_graph,
                       DataPropertyGraph* data_property_graph,
                       IndividualGraph* individual_graph,
                       AnonymousClassGraph* anonymous_graph) : class_graph_(class_graph),
                                                               object_property_graph_(object_property_graph),
                                                               data_property_graph_(data_property_graph),
                                                               individual_graph_(individual_graph),
                                                               anonymous_graph_(anonymous_graph)
  {
    for(auto* branch : other.all_branchs_)
    {
      auto* rule_branch = new RuleBranch(branch->value());
      all_branchs_.push_back(rule_branch);
    }
  }

  RuleBranch* RuleGraph::add(const std::size_t& value, Rule_t& rule)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<RuleBranch>::mutex_);

    const std::string rule_name = "rule_" + std::to_string(value);
    RuleBranch* rule_branch = new RuleBranch(rule_name);
    all_branchs_.push_back(rule_branch);

    // std::cout << "Computing antecedents : " << std::endl;
    for(auto& elem_antec : rule.antecedents)
    {
      if(elem_antec.first != nullptr)
        rule_branch->rule_antecedents_ = createRuleAtomList(elem_antec);
    }

    // std::cout << "Computing consequents : " << std::endl;
    for(auto& elem_conseq : rule.consequents)
    {
      if(elem_conseq.first != nullptr)
        rule_branch->rule_consequents_ = createRuleAtomList(elem_conseq);
    }

    return rule_branch;
  }

  RuleAtomList* RuleGraph::createRuleAtomList(std::pair<ontologenius::ExpressionMember_t*, std::vector<ontologenius::Variable_t>> rule_element)
  {
    auto* rule_atom = rule_element.first;
    auto rule_variable = rule_element.second;
    RuleAtomList* rule_list = new RuleAtomList();

    // if atom is a class expression, create an unamed anonymous class
    if((rule_atom->is_data_property) & (rule_atom->logical_type_ == logical_none) & (!rule_atom->is_complex))
    {
      // std::cout << "single data atom " << rule_atom->rest.toString() << std::endl;
      DataPropertyAtom* data_atom = createDataPropertyAtom(rule_atom, rule_variable);
      rule_list->data_atoms_.push_back(data_atom);
    }
    else if((rule_atom->logical_type_ == logical_none) & (!rule_atom->is_complex) & (rule_atom->rest.restriction_range.empty()))
    {
      // std::cout << "single object atom " << rule_atom->rest.toString() << std::endl;
      ObjectPropertyAtom* object_atom = createObjectPropertyAtom(rule_atom, rule_variable);
      rule_list->object_atoms_.push_back(object_atom);
    }
    else
    {
      // std::cout << " class atom  " << rule_atom->rest.toString() << std::endl;
      ClassAtom* class_atom = createClassAtom(rule_atom, rule_variable.front());
      rule_list->class_atoms_.push_back(class_atom);
    }

    return rule_list;
  }

  ClassAtom* RuleGraph::createClassAtom(ExpressionMember_t* class_member, Variable_t variable)
  {
    ClassAtom* class_atom = new ClassAtom();
    AnonymousClassElement* class_elem = new AnonymousClassElement();
    class_atom->class_expression = class_elem;

    if(class_member->logical_type_ != logical_none || class_member->oneof == true || class_member->is_complex == true)
    {
      // std::cout << "complex atom " << std::endl;
      size_t depth = 0;
      class_elem = anonymous_graph_->createTree(class_member, depth);
    }
    else if(!class_member->rest.property.empty())
    {
      // std::cout << "single restriction " << std::endl;
      class_elem = anonymous_graph_->createElement(class_member);
    }
    else
    {
      // std::cout << "simple class atom " << std::endl;
      class_elem->class_involved_ = class_graph_->findOrCreateBranch(class_member->rest.restriction_range);
    }

    if(variable.is_instantiated)
      class_atom->individual_involved = individual_graph_->findOrCreateBranch(variable.var_name);
    else
      class_atom->var = variable.var_name;

    return class_atom;
  }

  ObjectPropertyAtom* RuleGraph::createObjectPropertyAtom(ExpressionMember_t* property_member, std::vector<Variable_t> variable)
  {
    ObjectPropertyAtom* object_atom = new ObjectPropertyAtom();
    object_atom->object_property_expression = object_property_graph_->findOrCreateBranch(property_member->rest.property);

    if(variable.size() == 2)
    {
      // get argument 1
      if(variable.front().is_instantiated)
        object_atom->individual_involved_1 = individual_graph_->findOrCreateBranch(variable.front().var_name);
      else
        object_atom->var1 = variable.front().var_name;

      // get argument 2
      if(variable.back().is_instantiated)
        object_atom->individual_involved_2 = individual_graph_->findOrCreateBranch(variable.back().var_name);
      else
        object_atom->var2 = variable.back().var_name;
    }
    else
      return nullptr;

    return object_atom;
  }

  DataPropertyAtom* RuleGraph::createDataPropertyAtom(ExpressionMember_t* property_member, std::vector<Variable_t> variable)
  {
    DataPropertyAtom* data_atom = new DataPropertyAtom();
    data_atom->data_property_expression = data_property_graph_->findOrCreateBranch(property_member->rest.property);

    if(variable.size() == 2)
    {
      // get argument 1
      if(variable.front().is_instantiated)
        data_atom->individual_involved = individual_graph_->findOrCreateBranch(variable.front().var_name);
      else
        data_atom->var1 = variable.front().var_name;

      // get argument 2
      if(variable.back().is_datavalue)
        data_atom->datatype_involved = data_property_graph_->createLiteral(variable.back().var_name);
      else
        data_atom->var2 = variable.back().var_name;
    }
    else
      return nullptr;

    return data_atom;
  }

  AnonymousClassElement* RuleGraph::createElement(ExpressionMember_t* exp)
  {
    AnonymousClassElement* ano_element = new AnonymousClassElement();
    ano_element->is_complex = exp->is_complex;
    Restriction_t current_rest = exp->rest;

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
        ano_element->data_property_involved_ = data_property_graph_->findOrCreateBranch(current_rest.property);
      else
        ano_element->object_property_involved_ = object_property_graph_->findOrCreateBranch(current_rest.property);
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
        const std::string type_value = card_range.substr(card_range.find("#") + 1, -1);
        ano_element->card_.card_range_ = data_property_graph_->createLiteral(type_value);
      }
      else // object property
      {
        if(ano_element->card_.card_type_ == cardinality_value) // indiv
          ano_element->individual_involved_ = individual_graph_->findOrCreateBranch(card_range);
        else // class
          ano_element->class_involved_ = class_graph_->findOrCreateBranch(card_range);
      }
    }

    // ===============  Restriction range  (min, max, exactly )===================
    const std::string rest_range = current_rest.restriction_range;
    if(!rest_range.empty())
    {
      if(isIn("http://www.w3.org/", rest_range)) // literal node for complex data restriction (ClassX Eq to data_prop some (not(literal)))
      {
        const std::string type = split(rest_range, "#").back();
        ano_element->card_.card_range_ = data_property_graph_->createLiteral(type);
      }
      else if(exp->mother != nullptr && exp->mother->oneof) // individual node for oneOf (ClassX Eq to oneOf(indiv))
        ano_element->individual_involved_ = individual_graph_->findOrCreateBranch(rest_range);
      else
        ano_element->class_involved_ = class_graph_->findOrCreateBranch(rest_range); // class node for class only restriction (ClassX Eq to ClassY)
    }

    return ano_element;
  }

  AnonymousClassElement* RuleGraph::createTree(ExpressionMember_t* member_node, size_t& depth)
  {
    size_t local_depth = depth + 1;
    AnonymousClassElement* node = createElement(member_node);

    for(auto* child : member_node->child_members)
    {
      size_t child_depth = depth + 1;
      node->sub_elements_.push_back(createTree(child, child_depth));
      if(child_depth > local_depth)
        local_depth = child_depth;
    }

    depth = local_depth;

    return node;
  }

  void RuleGraph::printTree(AnonymousClassElement* ano_elem, size_t level, bool root) const
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

  std::string RuleGraph::toString(CardType_e value) const
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