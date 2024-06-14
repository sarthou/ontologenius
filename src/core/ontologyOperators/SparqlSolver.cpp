#include "ontologenius/core/ontologyOperators/SparqlSolver.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iostream>
#include <iterator>
#include <map>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontologyOperators/SparqlUtils.h"
#include "ontologenius/utils/String.h"

using namespace std::chrono;

namespace ontologenius {

  SparqlSolver::SparqlSolver() : onto_(nullptr),
                                 sparql_pattern_("SELECT\\s*(DISTINCT)?\\s*([^\n]+)([\\s\n]*)WHERE([\\s\n]*)(.*)")
  {
    operators_["NOT EXISTS"] = sparql_not_exists;
  }

  SparqlSolver::Iterator SparqlSolver::begin()
  {
    SparqlSolution_t initial_solution;

    std::smatch match;
    const bool has_matched = std::regex_match(query_, match, sparql_pattern_);
    if(has_matched)
    {
      std::string vars = match[2].str();
      removeChar(vars, {'\n', '\r'});
      removeUselessSpace(vars);
      const std::vector<std::string> vars_to_return = split(vars, " ");

      std::string pattern = getPattern(match[5].str());
      removeChar(pattern, {'\n', '\r'});

      initial_solution = getInitialSolutionStandard(pattern);
      for(const auto& var : vars_to_return)
      {
        if(var == "*")
        {
          initial_solution.solution_ = initial_solution.solution_full_;
          break;
        }
        else if(var.empty() == false)
          initial_solution.solution_[var.substr(1)] = "";
      }
    }
    else
    {
      std::cout << "[SparqlSolver] solve custom form" << std::endl;
      initial_solution = getInitialSolutionCustom();
      initial_solution.solution_ = initial_solution.solution_full_;
    }

    empty_solution_ = initial_solution;

    orderVariables(initial_solution);
    const int index = 0;
    const high_resolution_clock::time_point t1 = high_resolution_clock::now();
    stepDown(initial_solution, index);
    const high_resolution_clock::time_point t2 = high_resolution_clock::now();
    const duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "took " << time_span.count() * 1000 << "ms" << std::endl;

    return SparqlSolver::Iterator(initial_solution, this);
  }

  SparqlSolution_t SparqlSolver::getInitialSolutionStandard(const std::string& pattern)
  {
    SparqlSolution_t initial_solution;

    auto blocks = getBlocks(pattern);
    if(error_.empty() == false)
      return initial_solution;

    for(auto& block : blocks)
    {
      auto triplets = getTriplets(block.raw, ".");
      if(error_.empty() == false)
        return initial_solution;

      insertConstraints(triplets, initial_solution, block.op);
    }

    if(error_.empty())
      for(auto& constraint : initial_solution.variable_constraints_)
        initial_solution.solution_full_[constraint.first] = "";

    return initial_solution;
  }

  SparqlSolution_t SparqlSolver::getInitialSolutionCustom()
  {
    SparqlSolution_t initial_solution;

    auto triplets = getTriplets(query_, ",");
    if(triplets.empty() == false)
      insertConstraints(triplets, initial_solution, sparql_none);
    else
      error_ = "The query is malformed";

    if(error_.empty())
      for(auto& constraint : initial_solution.variable_constraints_)
        initial_solution.solution_full_[constraint.first] = "";

    return initial_solution;
  }

  void SparqlSolver::insertConstraints(const std::vector<strTriplet_t>& triplets, SparqlSolution_t& solution, SparqlOperator_e sparql_operator)
  {
    for(const auto& triplet : triplets)
    {
      if(triplet.object.is_variable)
      {
        auto constraint_it = solution.variable_constraints_.find(triplet.object.name);
        if(constraint_it == solution.variable_constraints_.end())
          constraint_it = solution.variable_constraints_.insert({triplet.object.name, SparqlVariableConstraint_t(triplet.object.name)}).first;

        SparqlConstraint_t constraint;
        constraint.operator_ = sparql_operator;
        constraint.triplet_ = triplet;
        constraint.subject_constraint_ = false;
        constraint.leaf_ = !triplet.subject.is_variable;
        constraint_it->second.constraints_.push_back(constraint);
        if(triplet.subject.is_variable)
          constraint_it->second.linked_variales_.insert(triplet.subject.name);
      }
      else if(triplet.subject.is_variable)
      {
        auto constraint_it = solution.variable_constraints_.find(triplet.subject.name);
        if(constraint_it == solution.variable_constraints_.end())
          constraint_it = solution.variable_constraints_.insert({triplet.subject.name, SparqlVariableConstraint_t(triplet.subject.name)}).first;

        SparqlConstraint_t constraint;
        constraint.operator_ = sparql_operator;
        constraint.triplet_ = triplet;
        constraint.subject_constraint_ = true;
        constraint.leaf_ = !triplet.object.is_variable;
        constraint_it->second.constraints_.push_back(constraint);
      }
      else if(triplet.predicat.is_variable)
        error_ = "The predicate of a triplet cannot be a variable";
      else
        error_ = "No variable found in the triplet";
    }
  }

  void SparqlSolver::orderVariables(SparqlSolution_t& solution)
  {
    std::unordered_map<std::string, std::unordered_set<std::string>> variables_links;
    for(auto& variable : solution.variable_constraints_)
      variables_links.insert({variable.first, variable.second.linked_variales_});

    solution.ordered_variables_.reserve(variables_links.size());
    while(variables_links.empty() == false)
    {
      std::string selected_var;
      for(auto& variable : variables_links)
        if(variable.second.empty())
        {
          solution.ordered_variables_.emplace_back(variable.first);
          selected_var = variable.first;
          break;
        }

      if(selected_var.empty() == false)
      {
        variables_links.erase(selected_var);
        for(auto& variable : variables_links)
          variable.second.erase(selected_var);
      }
      else
        std::cout << "-------TODO-------" << std::endl;
    }
  }

  void SparqlSolver::stepDown(SparqlSolution_t& solution, int index)
  {
    auto variable = solution.ordered_variables_[index];
    auto& candidates = solution.candidates_[variable];

    if(candidates.empty())
    {
      auto& constraints = solution.variable_constraints_.at(variable).constraints_;
      for(auto& constraint : constraints)
      {
        if(candidates.empty())
          candidates = solveTriplet(constraint.triplet_, solution.solution_full_);
        else
        {
          std::unordered_set<std::string> valids;
          for(const auto& candidate : candidates)
          {
            solution.solution_full_[variable] = candidate;
            if(solveTriplet(constraint.triplet_, solution.solution_full_).empty() == false)
              valids.insert(candidate);
          }
          candidates = std::move(valids);
          solution.solution_full_[variable] = "";
        }

        if(candidates.empty())
          break;
      }
    }

    const bool is_last = (index >= (int)(solution.ordered_variables_.size() - 1));
    while(candidates.empty() == false)
    {
      solution.solution_full_[variable] = *candidates.begin();
      candidates.erase(candidates.begin());

      if(is_last == false)
      {
        stepDown(solution, index + 1);
        auto next_var = solution.ordered_variables_[index + 1];
        if(solution.solution_full_[next_var].empty() == false)
          return;
      }
      else
        return;
    }

    solution.solution_full_[variable] = "";
  }

  void SparqlSolver::nextSolution(SparqlSolution_t& solution)
  {
    for(int i = (int)solution.ordered_variables_.size() - 1; i >= 0; i--)
    {
      auto variable = solution.ordered_variables_[i];
      if(solution.candidates_[variable].empty() == false)
      {
        stepDown(solution, i);
        if(solution.solution_full_[variable].empty() == false)
          break;
      }
      else
        solution.solution_full_[variable] = "";
    }
  }

  std::unordered_set<std::string> SparqlSolver::solveTriplet(strTriplet_t triplet, const std::map<std::string, std::string>& binding)
  {
    if(triplet.predicat.is_variable)
      error_ = "predicat can not be a variable in: " + toString(triplet);
    else if(triplet.predicat.name == "isA")
    {
      if(triplet.subject.is_variable && !triplet.object.is_variable)
      {
        auto var_it = binding.find(triplet.subject.name);
        if(var_it != binding.end())
          return getType(triplet, var_it->second);
        else
          return getType(triplet);
      }
      else if(!triplet.subject.is_variable && triplet.object.is_variable)
      {
        auto var_it = binding.find(triplet.object.name);
        if(var_it != binding.end())
          return getUp(triplet, var_it->second);
        else
          return getUp(triplet);
      }
      else if(triplet.subject.is_variable && triplet.object.is_variable)
      {
        auto var_it = binding.find(triplet.subject.name);
        if(var_it != binding.end())
        {
          triplet.subject.name = var_it->second;
          var_it = binding.find(triplet.object.name);
          if(var_it != binding.end())
            return getUp(triplet, var_it->second);
          else
            return getUp(triplet);
        }
        else
        {
          var_it = binding.find(triplet.object.name);
          if(var_it != binding.end())
          {
            triplet.object.name = var_it->second;
            return getType(triplet);
          }
          else
            error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
        }
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable";
    }
    else if(triplet.predicat.name == "hasLabel")
    {
      if(triplet.subject.is_variable && !triplet.object.is_variable)
      {
        auto var_it = binding.find(triplet.subject.name);
        if(var_it != binding.end())
          return find(triplet, var_it->second);
        else
          return find(triplet);
      }
      else if(!triplet.subject.is_variable && triplet.object.is_variable)
      {
        auto var_it = binding.find(triplet.object.name);
        if(var_it != binding.end())
          return getName(triplet, var_it->second);
        else
          return getName(triplet);
      }
      else if(triplet.subject.is_variable && triplet.object.is_variable)
      {
        auto var_it = binding.find(triplet.subject.name);
        if(var_it != binding.end())
        {
          triplet.subject.name = var_it->second;
          var_it = binding.find(triplet.object.name);
          if(var_it != binding.end())
            return getName(triplet, var_it->second);
          else
            return getName(triplet);
        }
        else
        {
          var_it = binding.find(triplet.object.name);
          if(var_it != binding.end())
          {
            triplet.object.name = var_it->second;
            return find(triplet);
          }
          else
            error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
        }
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable";
    }
    else if(triplet.subject.is_variable && !triplet.object.is_variable)
    {
      auto var_it = binding.find(triplet.subject.name);
      if(var_it != binding.end())
        return getFrom(triplet, var_it->second);
      else
        return getFrom(triplet);
    }
    else if(!triplet.subject.is_variable && triplet.object.is_variable)
    {
      auto var_it = binding.find(triplet.object.name);
      if(var_it != binding.end())
        return getOn(triplet, var_it->second);
      else
        return getOn(triplet);
    }
    else if(triplet.subject.is_variable && triplet.object.is_variable)
    {
      auto var_it = binding.find(triplet.subject.name);
      if(var_it != binding.end())
      {
        triplet.subject.name = var_it->second;
        var_it = binding.find(triplet.object.name);
        if(var_it != binding.end())
          return getOn(triplet, var_it->second);
        else
          return getOn(triplet);
      }
      else
      {
        var_it = binding.find(triplet.object.name);
        if(var_it != binding.end())
        {
          triplet.object.name = var_it->second;
          return getFrom(triplet);
        }
        else
          error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
      }
    }
    else
      error_ = "can not resolve query : " + toString(triplet) + " : No variable";

    return {};
  }

  std::unordered_set<std::string> SparqlSolver::getOn(const strTriplet_t& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.getOn(triplet.subject.name, triplet.predicat.name);
    if(selector.empty())
      return res;
    else if(res.find(selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
  }

  std::unordered_set<std::string> SparqlSolver::getFrom(const strTriplet_t& triplet, const std::string& selector)
  {
    if(selector.empty())
      return onto_->individual_graph_.getFrom(triplet.object.name, triplet.predicat.name);
    else
    {
      // Here we revert the problem as we know what we are expecting for.
      auto res = onto_->individual_graph_.getOn(selector, triplet.predicat.name);
      if(res.find(triplet.object.name) != res.end())
        return std::unordered_set<std::string>({selector});
      else
        return std::unordered_set<std::string>();
    }
  }

  std::unordered_set<std::string> SparqlSolver::getUp(const strTriplet_t& triplet, const std::string& selector)
  {
    if(selector.empty())
      return onto_->individual_graph_.getUp(triplet.subject.name);
    else
    {
      auto is = onto_->individual_graph_.getUp(triplet.subject.name);
      if(is.find(selector) != is.end())
        return std::unordered_set<std::string>({selector});
      else
        return std::unordered_set<std::string>();
    }
  }

  std::unordered_set<std::string> SparqlSolver::getType(const strTriplet_t& triplet, const std::string& selector)
  {
    if(selector.empty())
      return onto_->individual_graph_.getType(triplet.object.name);
    else
    {
      auto types = onto_->individual_graph_.getUp(selector);
      if(types.find(triplet.object.name) != types.end())
        return std::unordered_set<std::string>({selector});
      else
        return std::unordered_set<std::string>();
    }
  }

  std::unordered_set<std::string> SparqlSolver::find(const strTriplet_t& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.find<std::string>(triplet.object.name);
    if(selector.empty())
      return res;
    else if(res.find(selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
  }

  std::unordered_set<std::string> SparqlSolver::getName(const strTriplet_t& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.getNames(triplet.subject.name);
    if(selector.empty())
    {
      std::unordered_set<std::string> set_res;
      for(auto& r : res)
        set_res.insert(r);
      return set_res;
    }
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
  }

  std::string SparqlSolver::getPattern(const std::string& text)
  {
    const size_t begin_of_pattern = text.find('{');
    std::string res;
    getIn(begin_of_pattern, res, text, '{', '}');
    return res;
  }

  std::vector<SparqlBlock_t> SparqlSolver::getBlocks(std::string query)
  {
    removeUselessSpace(query);

    std::vector<SparqlBlock_t> res;
    std::unordered_map<std::string, std::string> tmp_blocks;
    std::unordered_set<std::string> blocks_id;
    size_t cpt = 0;
    size_t bracket_pose = 0;

    while((bracket_pose = query.find('{', bracket_pose)) != std::string::npos)
    {
      std::string text_in;
      const size_t end_pose = getIn(bracket_pose, text_in, query, '{', '}');
      if((end_pose == std::string::npos) || (end_pose == bracket_pose))
      {
        error_ = "Unclosed bracket in: " + query;
        return res;
      }
      const std::string mark = "__" + std::to_string(cpt);
      blocks_id.insert(mark);
      tmp_blocks.insert({mark, text_in});
      query.replace(bracket_pose, end_pose - bracket_pose + 1, mark);
      cpt++;
    }

    do
    {
      size_t first_block_pose = std::string::npos;
      std::string first_block;
      size_t first_keyword_pose = std::string::npos;
      std::string first_keyword;

      for(const auto& id : blocks_id)
      {
        const size_t tmp_pose = query.find(id);
        if(tmp_pose < first_block_pose)
        {
          first_block_pose = tmp_pose;
          first_block = id;
        }
      }

      for(auto& key : operators_)
      {
        const size_t tmp_pose = query.find(key.first);
        if(tmp_pose < first_keyword_pose)
        {
          first_keyword_pose = tmp_pose;
          first_keyword = key.first;
        }
      }

      if(first_block_pose == 0)
      {
        SparqlBlock_t block;
        block.raw = tmp_blocks[first_block];
        block.op = sparql_none;
        res.push_back(block);
        query = query.substr(first_block.size());
      }
      else if(first_keyword_pose == 0)
      {
        SparqlBlock_t block;
        block.raw = tmp_blocks[first_block];
        block.op = operators_[first_keyword];
        res.push_back(block);
        query = query.substr(first_block_pose + first_block.size());
      }
      else
      {
        const size_t min_pose = std::min(query.size(), std::min(first_keyword_pose, first_block_pose));
        SparqlBlock_t block;
        block.raw = query.substr(0, min_pose);
        block.op = sparql_none;
        res.push_back(block);
        query = query.substr(min_pose);
      }
      removeUselessSpace(query);
    } while(query.empty() == false);

    return res;
  }

  std::vector<strTriplet_t> SparqlSolver::getTriplets(const std::string& query, const std::string& delim)
  {
    const std::vector<std::string> sub_queries = split(query, delim);
    std::vector<strTriplet_t> sub_queries_triplet;
    try
    {
      std::transform(sub_queries.cbegin(), sub_queries.cend(), std::back_inserter(sub_queries_triplet), [](const auto& sub_query) { return getTriplet<std::string>(sub_query); });
    }
    catch(const std::string& msg)
    {
      error_ = msg;
    }

    return sub_queries_triplet;
  }

} // namespace ontologenius
