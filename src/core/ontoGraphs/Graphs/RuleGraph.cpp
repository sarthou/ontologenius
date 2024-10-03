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

    return data_atom;
  }

} // namespace ontologenius