#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

// #define DEBUG

namespace ontologenius {

  RuleGraph::RuleGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph,
                       IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph) : class_graph_(class_graph),
                                                                                                  object_property_graph_(object_property_graph),
                                                                                                  data_property_graph_(data_property_graph),
                                                                                                  individual_graph_(individual_graph),
                                                                                                  anonymous_graph_(anonymous_graph)
  {}

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

  RuleBranch* RuleGraph::add(const std::size_t& rule_id, Rule_t& rule)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<RuleBranch>::mutex_);

    const std::string rule_name = "rule_" + std::to_string(rule_id);
    RuleBranch* rule_branch = new RuleBranch(rule_name);
    all_branchs_.push_back(rule_branch);

    size_t elem_id = 0;

    for(auto& atom_antec : rule.antecedents)
    {
      if(atom_antec.first != nullptr)
      {
        rule_branch->rule_body_.push_back(createRuleAtomTriplet(rule_branch, atom_antec, rule_id, elem_id, true));
        rule_branch->atom_initial_order_.push_back(rule_branch->rule_body_.size() - 1);
      }
      elem_id++;
    }
    for(auto& atom_conseq : rule.consequents)
    {
      if(atom_conseq.first != nullptr)
        rule_branch->rule_head_.push_back(createRuleAtomTriplet(rule_branch, atom_conseq, rule_id, elem_id, false));
      elem_id++;
    }

    return rule_branch;
  }

  RuleTriplet_t RuleGraph::createRuleAtomTriplet(RuleBranch* rule_branch, const std::pair<ontologenius::ExpressionMember_t*, std::vector<ontologenius::Variable_t>>& rule_element, const size_t& rule_id, const size_t& elem_id, const bool& is_head)
  {
    auto* rule_atom = rule_element.first;
    auto rule_variable = rule_element.second;

    if((rule_atom->is_data_property) && (rule_atom->logical_type_ == logical_none) && (!rule_atom->is_complex))
    {
      if(is_head)
        rule_branch->involves_data_property = true;
      return createDataPropertyTriplet(rule_branch, rule_atom, rule_variable);
    }
    else if((rule_atom->logical_type_ == logical_none) && (!rule_atom->is_complex) && (rule_atom->rest.restriction_range.empty()))
    {
      if(is_head)
        rule_branch->involves_object_property = true;
      return createObjectPropertyTriplet(rule_branch, rule_atom, rule_variable);
    }
    else
    {
      if(is_head)
        rule_branch->involves_class = true;
      return createClassTriplet(rule_branch, rule_atom, rule_variable.front(), rule_id, elem_id);
    }
  }

  RuleTriplet_t RuleGraph::createClassTriplet(RuleBranch* rule_branch, ExpressionMember_t* class_member, const Variable_t& variable, const size_t& rule_id, const size_t& elem_id)
  {
    if(class_member->logical_type_ != logical_none || class_member->oneof == true || class_member->is_complex == true || !class_member->rest.property.empty())
    {
      RuleResource_t resource = getRuleResource(rule_branch, variable);                                            // create the variable
      AnonymousClassBranch* rule_ano_branch = anonymous_graph_->addHiddenRuleElem(rule_id, elem_id, class_member); // returns the newly created ano branch
      AnonymousClassElement* ano_elem = rule_ano_branch->ano_elems_.front();                                       // complex expression
      ClassBranch* hidden = rule_ano_branch->class_equiv_;

      return RuleTriplet_t(hidden, ano_elem, resource);
    }
    else
    {
      RuleResource_t resource = getRuleResource(rule_branch, variable); // create the variable
      ClassBranch* class_branch = class_graph_->findOrCreateBranch(class_member->rest.restriction_range);

      return RuleTriplet_t(class_branch, resource);
    }
  }

  RuleTriplet_t RuleGraph::createObjectPropertyTriplet(RuleBranch* rule_branch, ExpressionMember_t* property_member, const std::vector<Variable_t>& variable)
  {
    RuleResource_t resource_from = getRuleResource(rule_branch, variable.front());
    ObjectPropertyBranch* property = object_property_graph_->findOrCreateBranch(property_member->rest.property);
    RuleResource_t resource_on = getRuleResource(rule_branch, variable.back());

    return RuleTriplet_t(resource_from, property, resource_on);
  }

  RuleTriplet_t RuleGraph::createDataPropertyTriplet(RuleBranch* rule_branch, ExpressionMember_t* property_member, const std::vector<Variable_t>& variable)
  {
    RuleResource_t resource_from = getRuleResource(rule_branch, variable.front());
    DataPropertyBranch* property = data_property_graph_->findOrCreateBranch(property_member->rest.property);
    RuleResource_t resource_on = getRuleResource(rule_branch, variable.back());

    return RuleTriplet_t(resource_from, property, resource_on);
  }

  RuleResource_t RuleGraph::getRuleResource(RuleBranch* rule_branch, const Variable_t& variable)
  {
    if(variable.is_instantiated == true)
    {
      // individual
      IndividualBranch* involved_indiv = individual_graph_->findOrCreateBranch(variable.var_name);
      RuleResource_t resource(involved_indiv);
      setVariableIndex(rule_branch, resource);
      return resource;
    }
    else if(variable.is_datavalue == true)
    {
      // literal
      LiteralNode* involved_datatype = data_property_graph_->createLiteral(variable.var_name); // invalid use of incomplete type ‘class ontologenius::DataPropertyGraph’
      RuleResource_t resource(involved_datatype);
      setVariableIndex(rule_branch, resource);
      return resource;
    }
    else
    {
      // variable
      RuleResource_t resource(variable.var_name);
      setVariableIndex(rule_branch, resource);
      return resource;
    }
  }

  void RuleGraph::setVariableIndex(RuleBranch* rule_branch, RuleResource_t& resource)
  {
    auto var_it = rule_branch->variables_.find(resource.name);
    if(var_it == rule_branch->variables_.end()) // if the variable doesn't already exist in the variables_ vector of the rule
    {
      int64_t index = std::int64_t(rule_branch->variables_.size());
      rule_branch->variables_[resource.name] = index;
      resource.variable_id = index;
      rule_branch->to_variables_.push_back(resource.name);
      variable_names_.insert(resource.name); // to rewrite the variables
    }
    else
      resource.variable_id = var_it->second; // if it already does, we just link it with its
  }

} // namespace ontologenius