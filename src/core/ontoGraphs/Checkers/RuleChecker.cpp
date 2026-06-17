#include "ontologenius/core/ontoGraphs/Checkers/RuleChecker.h"

#include <cstddef>
#include <cstdint>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

#define USE_NONE 0
#define USE_INDIVIDUAL 1 << 0
#define USE_DATATYPE 1 << 1
#define USE_BOTH (USE_INDIVIDUAL | USE_DATATYPE)

namespace ontologenius {

  size_t RuleChecker::check()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(graphs_->rules_.mutex_);

    for(auto* rule : graphs_->rules_.all_branchs_)
      checkRuleDisjoint(rule);

    is_analysed = true;
    printStatus();

    return getErrors();
  }

  void RuleChecker::checkRuleDisjoint(RuleBranch* branch)
  {
    current_rule_ = branch->getRule();

    std::unordered_map<std::string, uint8_t> all_arguments;
    std::unordered_map<std::string, std::pair<std::unordered_set<ClassBranch*>, std::unordered_set<LiteralType*>>> variables_types;
    for(auto& atom : branch->rule_body_)
      checkAtom(atom, variables_types, all_arguments);

    for(auto& atom : branch->rule_head_)
      checkAtom(atom, variables_types, all_arguments);

    for(auto& arg : all_arguments)
    {
      if(arg.second == USE_BOTH)
      {
        const std::string err_base = "In rule " + current_rule_ + ": error related to variable " + arg.first + " because ";
        printError(err_base + " it is used both as an individual and a data value.");
      }
    }
  }

  void RuleChecker::checkAtom(const RuleTriplet_t& atom,
                              std::unordered_map<std::string, std::pair<std::unordered_set<ClassBranch*>, std::unordered_set<LiteralType*>>>& variables_types,
                              std::unordered_map<std::string, uint8_t>& all_arguments)
  {
    switch(atom.atom_type_)
    {
    case rule_atom_class:
      checkClassAtom(atom, variables_types);
      setArgument(atom.arguments.at(0), all_arguments, true);
      break;
    case rule_atom_object:
      checkObjectPropertyAtom(atom, variables_types);
      setArgument(atom.arguments.at(0), all_arguments, true);
      setArgument(atom.arguments.at(1), all_arguments, true);
      break;
    case rule_atom_data:
      checkDataPropertyAtom(atom, variables_types);
      setArgument(atom.arguments.at(0), all_arguments, true);
      setArgument(atom.arguments.at(1), all_arguments, false);
      break;
    case rule_atom_builtin:
      checkBuiltinAtom(atom, variables_types);
      for(const auto& arg : atom.arguments)
        setArgument(arg, all_arguments, false);
      break;
    default:
      break;
    }
  }

  void RuleChecker::setArgument(const RuleArgument_t& arg, std::unordered_map<std::string, uint8_t>& all_arguments, bool individual_usage)
  {
    auto arg_it = all_arguments.find(arg.name);
    if(arg_it == all_arguments.end())
    {
      arg_it = all_arguments.emplace(arg.name, USE_NONE).first;
    }

    if(arg.datatype_value != nullptr)
      arg_it->second |= USE_DATATYPE;
    if(arg.indiv_value != nullptr)
      arg_it->second |= USE_INDIVIDUAL;

    arg_it->second |= (individual_usage ? USE_INDIVIDUAL : USE_DATATYPE);
  }

  void RuleChecker::checkClassAtom(const RuleTriplet_t& atom, std::unordered_map<std::string, std::pair<std::unordered_set<ClassBranch*>, std::unordered_set<LiteralType*>>>& variables_types)
  {
    if(atom.class_predicate != nullptr)
    {
      if(atom.anonymous_element != nullptr)
      {
        const auto& anonymous_trees = atom.anonymous_element->ano_trees_;
        if(anonymous_trees.empty() == false)
        {
          auto* anonymous_root = anonymous_trees.front()->root_node_;
          (void)anonymous_root; // Todo extract the classes from the class expression
        }
      }
      else
      {
        const auto& classes = atom.class_predicate->mothers_.relations;
        std::string err_inheritance = check(classes, variables_types[atom.arguments.at(0).name].first);

        if(err_inheritance.empty() == false)
          raiseError(atom.arguments.at(0), " using class " + atom.class_predicate->value() + ", whcih is disjoint with other constraints (disjointness between " + err_inheritance + ").");
      }
    }
  }

  void RuleChecker::checkObjectPropertyAtom(const RuleTriplet_t& atom, std::unordered_map<std::string, std::pair<std::unordered_set<ClassBranch*>, std::unordered_set<LiteralType*>>>& variables_types)
  {
    if(atom.object_predicate != nullptr)
    {
      const auto& domains = atom.object_predicate->domains_;
      const auto& ranges = atom.object_predicate->ranges_;

      std::string err_domain = check(domains, variables_types[atom.arguments.at(0).name].first);
      std::string err_range = check(ranges, variables_types[atom.arguments.at(1).name].first);

      if(err_domain.empty() == false)
        raiseError(atom.arguments.at(0), " using property " + atom.object_predicate->value() + ", its domain is disjoint with other constraints (disjointness between " + err_domain + ").");

      if(err_range.empty() == false)
        raiseError(atom.arguments.at(1), " using property " + atom.object_predicate->value() + ", its range is disjoint with other constraints (disjointness between " + err_domain + ").");
    }
  }

  void RuleChecker::checkDataPropertyAtom(const RuleTriplet_t& atom, std::unordered_map<std::string, std::pair<std::unordered_set<ClassBranch*>, std::unordered_set<LiteralType*>>>& variables_types)
  {
    if(atom.data_predicate != nullptr)
    {
      const auto& domains = atom.data_predicate->domains_;
      const auto& ranges = atom.data_predicate->ranges_;

      std::string err_domain = check(domains, variables_types[atom.arguments.at(0).name].first);
      std::string err_range = check(ranges, variables_types[atom.arguments.at(1).name].second);

      if(err_domain.empty() == false)
        raiseError(atom.arguments.at(0), " using property " + atom.data_predicate->value() + ", its domain is disjoint with other constraints (disjointness between " + err_domain + ").");

      if(err_range.empty() == false)
        raiseError(atom.arguments.at(0), " using property " + atom.data_predicate->value() + ", its range is disjoint with other constraints (disjointness between " + err_domain + ").");
    }
  }

  void RuleChecker::checkBuiltinAtom(const RuleTriplet_t& atom, std::unordered_map<std::string, std::pair<std::unordered_set<ClassBranch*>, std::unordered_set<LiteralType*>>>& variables_types)
  {
    for(const auto& argument : atom.arguments)
    {
      for(const auto& again_argument : atom.arguments)
      {
        if(again_argument.is_variable == false)
        {
          std::string err = check({again_argument.datatype_value->type_}, variables_types[argument.name].second);
          if(err.empty() == false)
          {
            raiseError(argument, " using builtin " + atom.builtinToString() + ", its arguments are disjoint with other constraints (disjointness between " + err + ").");
            return;
          }
        }
      }
    }
  }

  std::string RuleChecker::check(const std::vector<ClassElement>& classes, std::unordered_set<ClassBranch*>& variables_classes)
  {
    std::string err;
    for(const auto& class_element : classes)
    {
      if(variables_classes.insert(class_element.elem).second)
      {
        if(err.empty()) // We only want to take the first error but we need to insert all the classes
        {
          std::unordered_set<ClassBranch*> disjoints;
          graphs_->classes_.getDisjoint(class_element.elem, disjoints);
          const ClassBranch* intersection = graphs_->classes_.firstIntersection(variables_classes, disjoints);
          if(intersection != nullptr)
            err = class_element.elem->value() + " and " + intersection->value();
        }
      }
    }
    return err;
  }

  std::string RuleChecker::check(const std::vector<LiteralType*>& types, std::unordered_set<LiteralType*>& variables_types) // todo: use a proper comparison of data types
  {
    if(variables_types.empty())
    {
      for(auto* type : types)
        variables_types.insert(type);
    }
    else
    {
      for(auto* type : types)
      {
        auto type_it = variables_types.find(type);
        if(type_it == variables_types.end())
        {
          std::string err = type->value() + " and ";
          for(auto* err_type : variables_types)
            err += err_type->value();
          return err;
        }
      }
    }
    return "";
  }

  void RuleChecker::raiseError(const RuleArgument_t& var, const std::string& msg)
  {
    const std::string err_base = "In rule " + current_rule_ + ": error related to variable " + var.name + " because ";
    printError(err_base + msg);
  }

} // namespace ontologenius