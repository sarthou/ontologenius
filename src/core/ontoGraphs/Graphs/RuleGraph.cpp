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
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntologyGraphs.h"
#include "ontologenius/utils/String.h"

// #define DEBUG

namespace ontologenius {

  RuleGraph::RuleGraph(OntologyGraphs* graphs) : graphs_(graphs)
  {}

  RuleGraph::RuleGraph(const RuleGraph& other,
                       OntologyGraphs* graphs) : // Graph copy constructor is not called as RuleBranch need more advanced copy
                                                 graphs_(graphs)
  {
    all_branchs_.reserve(other.all_branchs_.size());
    for(auto* branch : other.all_branchs_)
    {
      auto* rule_branch = new RuleBranch(branch->value(), branch->getRule());
      all_branchs_.push_back(rule_branch);
    }
  }

  RuleBranch* RuleGraph::add(const RuleDescriptor_t& rule)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<RuleBranch>::mutex_);

    const std::string rule_name = "rule_" + std::to_string(all_branchs_.size());
    RuleBranch* rule_branch = new RuleBranch(rule_name, rule.rule_str);
    all_branchs_.push_back(rule_branch);

    // add comment
    rule_branch->setCommentDictionary(rule.comments_);

    size_t elem_id = 0;

    for(const auto& atom : rule.antecedents)
    {
      rule_branch->atom_initial_order_.push_back(rule_branch->rule_body_.size());
      rule_branch->rule_body_.push_back(createRuleAtomTriplet(rule_branch, atom, elem_id, true));
      elem_id++;
    }

    for(const auto& atom : rule.consequents)
    {
      rule_branch->rule_head_.push_back(createRuleAtomTriplet(rule_branch, atom, elem_id, false));
      elem_id++;
    }

    return rule_branch;
  }

  RuleTriplet_t RuleGraph::createRuleAtomTriplet(RuleBranch* rule_branch, const std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>& atom, const size_t& elem_id, const bool& is_head)
  {
    RuleTriplet_t rule_triplet;

    const auto& rule_atom = atom.first;

    for(const auto& variable : atom.second)
      rule_triplet.arguments.push_back(getRuleArgument(rule_branch, variable));

    rule_triplet.atom_type_ = rule_atom.type;
    switch(rule_atom.type)
    {
    case RuleAtomType_e::rule_atom_builtin:
      rule_triplet.builtin = rule_atom.builtin;
      break;
    case RuleAtomType_e::rule_atom_data:
      if(is_head)
        rule_branch->involves_data_property = true;

      rule_triplet.data_predicate = graphs_->data_properties_.findOrCreateBranch(rule_atom.resource_value);
      break;
    case RuleAtomType_e::rule_atom_object:
      if(is_head)
        rule_branch->involves_object_property = true;

      rule_triplet.object_predicate = graphs_->object_properties_.findOrCreateBranch(rule_atom.resource_value);
      break;
    case RuleAtomType_e::rule_atom_class:
      if(is_head)
        rule_branch->involves_class = true;
      if(rule_atom.class_expression != nullptr)
      {
        EquivalentClassDescriptor_t class_descriptor;
        class_descriptor.class_name = rule_branch->value() + "_" + std::to_string(elem_id);
        class_descriptor.expression_members.push_back(rule_atom.class_expression);
        rule_triplet.anonymous_element = graphs_->anonymous_classes_.add(class_descriptor, true); // returns the newly created ano branch
        rule_triplet.class_predicate = rule_triplet.anonymous_element->class_equiv_;
      }
      else
        rule_triplet.class_predicate = graphs_->classes_.findOrCreateBranch(rule_atom.resource_value);
      break;
    default:
      break;
    }

    return rule_triplet;
  }

  RuleArgument_t RuleGraph::getRuleArgument(RuleBranch* rule_branch, const RuleVariableDescriptor_t& variable)
  {
    if(variable.is_instanciated == false)
    {
      RuleArgument_t resource(variable.name);
      setVariableIndex(rule_branch, resource);
      return resource;
    }
    else if(variable.datatype)
    {
      LiteralNode* involved_datatype = graphs_->literals_.findOrCreate(variable.name);
      RuleArgument_t resource(involved_datatype);
      return resource;
    }
    else
    {
      IndividualBranch* involved_indiv = graphs_->individuals_.findOrCreateBranch(variable.name);
      RuleArgument_t resource(involved_indiv);
      return resource;
    }
  }

  void RuleGraph::setVariableIndex(RuleBranch* rule_branch, RuleArgument_t& resource)
  {
    auto var_it = rule_branch->variables_.find(resource.name);
    if(var_it == rule_branch->variables_.end()) // if the variable doesn't already exist in the variables_ vector of the rule
    {
      int64_t index = static_cast<std::int64_t>(rule_branch->variables_.size());
      rule_branch->variables_[resource.name] = index;
      resource.variable_id = index;
      variable_names_.insert(resource.name); // to rewrite the variables
    }
    else
      resource.variable_id = var_it->second; // if it already does, we just link it with its
  }

  void RuleGraph::deepCopy(const RuleGraph& other)
  {
    for(size_t i = 0; i < other.all_branchs_.size(); i++)
      cpyBranch(other.all_branchs_[i], all_branchs_[i]);
  }

  void RuleGraph::cpyBranch(RuleBranch* old_branch, RuleBranch* new_branch)
  {
    new_branch->nb_updates_ = old_branch->nb_updates_;
    new_branch->setUpdated(old_branch->isUpdated());
    new_branch->flags_ = old_branch->flags_;

    new_branch->comments_ = old_branch->comments_;

    // addon
    new_branch->involves_class = old_branch->involves_class;
    new_branch->involves_object_property = old_branch->involves_object_property;
    new_branch->involves_data_property = old_branch->involves_data_property;

    for(const auto& elem : old_branch->rule_body_)
      new_branch->rule_body_.emplace_back(createCopyRuleTriplet(elem, new_branch));

    for(const auto& elem : old_branch->rule_head_)
      new_branch->rule_head_.emplace_back(createCopyRuleTriplet(elem, new_branch));

    new_branch->atom_initial_order_ = old_branch->atom_initial_order_;
  }

  RuleTriplet_t RuleGraph::createCopyRuleTriplet(const RuleTriplet_t& old_triplet, RuleBranch* new_branch)
  {
    RuleTriplet_t new_triplet;
    new_triplet.atom_type_ = old_triplet.atom_type_;

    if(old_triplet.class_predicate != nullptr)
      new_triplet.class_predicate = graphs_->classes_.container_.find(old_triplet.class_predicate->value());

    if(old_triplet.anonymous_element != nullptr)
      new_triplet.anonymous_element = graphs_->anonymous_classes_.container_.find(old_triplet.anonymous_element->value());

    if(old_triplet.data_predicate != nullptr)
      new_triplet.data_predicate = graphs_->data_properties_.container_.find(old_triplet.data_predicate->value());

    if(old_triplet.object_predicate != nullptr)
      new_triplet.object_predicate = graphs_->object_properties_.container_.find(old_triplet.object_predicate->value());

    new_triplet.builtin = old_triplet.builtin;

    for(const auto& arg : old_triplet.arguments)
    {
      RuleArgument_t new_arg;
      new_arg.name = arg.name;
      new_arg.variable_id = arg.variable_id;
      new_arg.is_variable = arg.is_variable;
      if(arg.indiv_value != nullptr)
        new_arg.indiv_value = graphs_->individuals_.container_.find(arg.indiv_value->value());
      if(arg.datatype_value != nullptr)
        new_arg.datatype_value = graphs_->literals_.find(arg.datatype_value->value());

      new_triplet.arguments.push_back(new_arg);

      setVariableIndex(new_branch, new_arg);
    }

    return new_triplet;
  }

} // namespace ontologenius