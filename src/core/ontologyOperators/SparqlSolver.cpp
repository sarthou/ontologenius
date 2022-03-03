#include "ontologenius/core/ontologyOperators/SparqlSolver.h"

#include "ontologenius/utils/String.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius
{
  SparqlSolver::SparqlSolver() : sparql_pattern_("SELECT\\s*(DISTINCT)?\\s*([^\n]+)([\\s\n]*)WHERE([\\s\n]*)(.*)")
  {
    onto_ = nullptr;
    error_ = "";

    operators_["NOT EXISTS"] = sparql_not_exists;
  }

  SparqlSolver::iterator SparqlSolver::begin()
  {
    SparqlSolution_t initial_solution;

    std::smatch match;
    if (std::regex_match(query_, match, sparql_pattern_))
    {
      std::string vars = match[2].str();
      removeChar(vars, {'\n', '\r'});
      removeUselessSpace(vars);
      std::vector<std::string> vars_to_return = split(vars, " ");

      std::string pattern = getPattern(match[5].str());
      removeChar(pattern, {'\n', '\r'});

      initial_solution = getInitialSolutionStandard(pattern);
      for(auto& var : vars_to_return)
      {
        if(var == "*")
        {
          initial_solution.solution_ = initial_solution.solution_full_;
          break;
        }
        else if(var != "")
          initial_solution.solution_[var.substr(1)] = "";
      }
        
    }
    else
    {
      initial_solution = getInitialSolutionCustom();
      initial_solution.solution_ = initial_solution.solution_full_;
    }

    empty_solution_ = initial_solution;

    orderVariables(initial_solution);
    int index = 0;
    stepDown(initial_solution, index);

    return SparqlSolver::iterator(initial_solution, this);
  }

  const SparqlSolver::iterator SparqlSolver::cbegin()
  {
    return begin();
  }

  SparqlSolution_t SparqlSolver::getInitialSolutionStandard(const std::string& pattern)
  {
    SparqlSolution_t initial_solution;

    auto blocks = getBlocks(pattern);
    if(error_ != "")
      return initial_solution;

    for(auto& block : blocks)
    {
       auto triplets = getTriplets(block.raw, ".");
      if(error_ != "")
        return initial_solution;

      insertConstraints(triplets, initial_solution, block.op);
    }

    if(error_ == "")
      for(auto& constraint : initial_solution.variable_constraints_)
        initial_solution.solution_full_[constraint.first] = "";

    return initial_solution;
  }

  SparqlSolution_t SparqlSolver::getInitialSolutionCustom()
  {
    SparqlSolution_t initial_solution;

    auto triplets = getTriplets(query_, ",");
    if(triplets.size())
      insertConstraints(triplets, initial_solution, sparql_none);
    else
      error_ = "The query is malformed";

    if(error_ != "")
      for(auto& constraint : initial_solution.variable_constraints_)
        initial_solution.solution_full_[constraint.first] = "";

    return initial_solution;
  }
 
  void SparqlSolver::insertConstraints(const std::vector<triplet_t>& triplets, SparqlSolution_t& solution, SparqlOperator_e sparql_operator)
  {
    for(auto& triplet : triplets)
    {
      if(triplet.object.variable)
      {
        auto constraint_it = solution.variable_constraints_.find(triplet.object.name);
        if(constraint_it == solution.variable_constraints_.end())
          constraint_it = solution.variable_constraints_.insert({triplet.object.name, SparqlVariableConstraint_t(triplet.object.name)}).first;

        SparqlConstraint_t constraint;
        constraint.operator_ = sparql_operator;
        constraint.triplet_ = triplet;
        constraint.subject_constraint_ = false;
        constraint.leaf_ = !triplet.subject.variable;
        constraint_it->second.constraints_.push_back(constraint);
        if(triplet.subject.variable)
          constraint_it->second.linked_variales_.insert(triplet.subject.name);
      }
      else if(triplet.subject.variable)
      {
        auto constraint_it = solution.variable_constraints_.find(triplet.subject.name);
        if(constraint_it == solution.variable_constraints_.end())
          constraint_it = solution.variable_constraints_.insert({triplet.subject.name, SparqlVariableConstraint_t(triplet.subject.name)}).first;

        SparqlConstraint_t constraint;
        constraint.operator_ = sparql_operator;
        constraint.triplet_ = triplet;
        constraint.subject_constraint_ = true;
        constraint.leaf_ = !triplet.object.variable;
        constraint_it->second.constraints_.push_back(constraint);
      }
      else if(triplet.predicat.variable)
        error_ = "The predicate of a triplet cannot be a variable";
      else
        error_ = "No variable found in the triplet";
    }
  }

  void SparqlSolver::orderVariables(SparqlSolution_t& solution)
  {
    std::map<std::string, std::unordered_set<std::string>> variables_links;
    for(auto& variable : solution.variable_constraints_)
      variables_links[variable.first] = variable.second.linked_variales_;

    std::string selected_var = "";
    while(variables_links.size())
    {
      selected_var = "";
      for(auto& variable : variables_links)
        if(variable.second.size() == 0)
        {
          solution.ordered_variables_.push_back(variable.first);
          selected_var = variable.first;
          break;
        }

      if(selected_var != "")
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

    if(candidates.size() == 0)
    {
      auto& constraints = solution.variable_constraints_.at(variable).constraints_;
      for(auto& constraint : constraints)
      {
        if(candidates.size() == 0)
          candidates = solveTriplet(constraint.triplet_, solution.solution_full_);
        else
        {
          std::unordered_set<std::string> valids;
          for(auto& candidate : candidates)
          {
            solution.solution_full_[variable] = candidate;
            if(solveTriplet(constraint.triplet_, solution.solution_full_).size())
              valids.insert(candidate);
          }
          candidates = std::move(valids);
          solution.solution_full_[variable] = "";
        }

        if(candidates.size() == 0)
          break;
      }
    }

    while(candidates.size())
    {
      solution.solution_full_[variable] = *candidates.begin();
      candidates.erase(candidates.begin());

      if(index < solution.ordered_variables_.size() - 1)
      {
        stepDown(solution, index+1);
        auto next_var = solution.ordered_variables_[index+1];
        if(solution.solution_full_[next_var] != "")
          return;
      }
      else
        return;
    }

    solution.solution_full_[variable] = "";
  }

  void SparqlSolver::nextSolution(SparqlSolution_t& solution)
  {
    for(int i = solution.ordered_variables_.size() - 1; i >= 0; i--)
    {
      auto variable = solution.ordered_variables_[i];
      if(solution.candidates_[variable].size() != 0)
      {
        stepDown(solution, i);
        if(solution.solution_full_[variable] != "")
          break;
      }
      else
        solution.solution_full_[variable] = "";
    }
  }

  std::unordered_set<std::string> SparqlSolver::solveTriplet(triplet_t triplet, const std::map<std::string, std::string>& binding)
  {
    std::unordered_set<std::string> local_res;
    std::string var_name;

    if(triplet.predicat.variable)
      error_ = "predicat can not be a variable in: " + toString(triplet);
    else if(triplet.predicat.name == "isA")
    {
      if(triplet.subject.variable && !triplet.object.variable)
      {
        var_name = triplet.subject.name;
        if(binding.find(var_name) != binding.end())
          local_res = getType(triplet, binding.at(var_name));
        else
          local_res = getType(triplet);
      }
      else if(!triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.object.name;
        if(binding.find(var_name) != binding.end())
          local_res = getUp(triplet, binding.at(var_name));
        else
          local_res = getUp(triplet);
      }
      else if(triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.subject.name;
        if(binding.find(var_name) != binding.end())
        {
          triplet.subject.name = binding.at(triplet.subject.name);
          var_name = triplet.object.name;
          if(binding.find(var_name) != binding.end())
            local_res = getUp(triplet, binding.at(var_name));
          else
            local_res = getUp(triplet);
        }
        else if(binding.find(triplet.object.name) != binding.end())
        {
          triplet.object.name = binding.at(triplet.object.name);
          local_res = getType(triplet);
        }
        else
          error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable";
    }
    else if(triplet.predicat.name == "hasLabel")
    {
      if(triplet.subject.variable && !triplet.object.variable)
      {
        var_name = triplet.subject.name;
        if(binding.find(var_name) != binding.end())
          local_res = find(triplet, binding.at(var_name));
        else
          local_res = find(triplet);
      }
      else if(!triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.object.name;
        if(binding.find(var_name) != binding.end())
          local_res = getName(triplet, binding.at(var_name));
        else
          local_res = getName(triplet);
      }
      else if(triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.subject.name;
        if(binding.find(var_name) != binding.end())
        {
          triplet.subject.name = binding.at(triplet.subject.name);
          var_name = triplet.object.name;
          if(binding.find(var_name) != binding.end())
            local_res = getName(triplet, binding.at(var_name));
          else
            local_res = getName(triplet);
        }
        else if(binding.find(triplet.object.name) != binding.end())
        {
          triplet.object.name = binding.at(triplet.object.name);
          local_res = find(triplet);
        }
        else
          error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable";
    }
    else if(triplet.subject.variable && !triplet.object.variable)
    {
      var_name = triplet.subject.name;
      if(binding.find(var_name) != binding.end())
        local_res = getFrom(triplet, binding.at(var_name));
      else
        local_res = getFrom(triplet);
    }
    else if(!triplet.subject.variable && triplet.object.variable)
    {
      var_name = triplet.object.name;
      if(binding.find(var_name) != binding.end())
        local_res = getOn(triplet, binding.at(var_name));
      else
        local_res = getOn(triplet);
    }
    else if(triplet.subject.variable && triplet.object.variable)
    {
      var_name = triplet.subject.name;
      if(binding.find(var_name) != binding.end())
      {
        triplet.subject.name = binding.at(triplet.subject.name);
        var_name = triplet.object.name;
        if(binding.find(var_name) != binding.end())
          local_res = getOn(triplet, binding.at(var_name));
        else
          local_res = getOn(triplet);
      }
      else if(binding.find(triplet.object.name) != binding.end())
      {
        triplet.object.name = binding.at(triplet.object.name);
        local_res = getFrom(triplet);
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
    }
    else
      error_ = "can not resolve query : " + toString(triplet) + " : No variable";

    return local_res;
  }

  std::unordered_set<std::string> SparqlSolver::getOn(const triplet_t& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.getOn(triplet.subject.name, triplet.predicat.name);
    if(selector == "")
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
  }

  std::unordered_set<std::string> SparqlSolver::getFrom(const triplet_t& triplet, const std::string& selector)
  {
    if(selector == "")
      return onto_->individual_graph_.getFrom(triplet.object.name, triplet.predicat.name);
    else
    {
      // Here we revert the problem as we know what we are expecting for.
      auto res = onto_->individual_graph_.getOn(selector, triplet.predicat.name);
      if(std::find(res.begin(), res.end(), triplet.object.name) != res.end())
        return std::unordered_set<std::string>({selector});
      else
        return std::unordered_set<std::string>();
    }
  }

  std::unordered_set<std::string> SparqlSolver::getUp(const triplet_t& triplet, const std::string& selector)
  {
    if(selector == "")
      return onto_->individual_graph_.getUp(triplet.subject.name);
    else
    {
      auto is = onto_->individual_graph_.getUp(triplet.subject.name);
      if(std::find(is.begin(), is.end(), selector) != is.end())
        return std::unordered_set<std::string>({selector});
      else
        return std::unordered_set<std::string>();
    }
  }

  std::unordered_set<std::string> SparqlSolver::getType(const triplet_t& triplet, const std::string& selector)
  {
    if(selector == "")
      return onto_->individual_graph_.getType(triplet.object.name);
    else
    {
      auto types = onto_->individual_graph_.getUp(selector);
      if(std::find(types.begin(), types.end(), triplet.object.name) != types.end())
        return std::unordered_set<std::string>({selector});
      else
        return std::unordered_set<std::string>();
    }
  }

  std::unordered_set<std::string> SparqlSolver::find(const triplet_t& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.find(triplet.object.name);
    if(selector == "")
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
  }

  std::unordered_set<std::string> SparqlSolver::getName(const triplet_t& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.getNames(triplet.subject.name);
    if(selector == "")
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
    size_t begin_of_pattern = text.find("{");
    std::string res;
    getIn(begin_of_pattern, res, text, '{', '}');
    return res;
  }

  std::vector<SparqlBlock_t> SparqlSolver::getBlocks(std::string query)
  {
    removeUselessSpace(query);

    std::vector<SparqlBlock_t> res;
    std::map<std::string, std::string> tmp_blocks;
    std::set<std::string> blocks_id;
    size_t cpt = 0;
    size_t bracket_pose = 0;

    while((bracket_pose = query.find("{", bracket_pose)) != std::string::npos)
    {
      std::string text_in;
      size_t end_pose = getIn(bracket_pose, text_in, query, '{', '}');
      if((end_pose == std::string::npos) || (end_pose == bracket_pose))
      {
        error_ = "Unclosed bracket in: " + query;
        return res;
      }
      std::string mark = "__" + std::to_string(cpt);
      blocks_id.insert(mark);
      tmp_blocks[mark] = text_in;
      query.replace(bracket_pose, end_pose - bracket_pose + 1, mark);
      cpt++;
    }

    do
    {
      size_t first_block_pose = std::string::npos;
      std::string first_block = "";
      size_t first_keyword_pose = std::string::npos;
      std::string first_keyword = "";

      for(auto& id : blocks_id)
      {
        size_t tmp_pose = query.find(id);
        if(tmp_pose < first_block_pose)
        {
          first_block_pose = tmp_pose;
          first_block = id;
        } 
      }

      for(auto& key : operators_)
      {
        size_t tmp_pose = query.find(key.first);
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
        size_t min_pose = std::min(query.size(), std::min(first_keyword_pose, first_block_pose));
        SparqlBlock_t block;
        block.raw = query.substr(0, min_pose);
        block.op = sparql_none;
        res.push_back(block);
        query = query.substr(min_pose);
      }
      removeUselessSpace(query);
    }
    while(query != "");

    return res;
  }

  std::vector<triplet_t> SparqlSolver::getTriplets(const std::string& query, const std::string& delim)
  {
    std::vector<std::string> sub_queries = split(query, delim);
    std::vector<triplet_t> sub_queries_triplet;
    try
    {
      for(auto& q : sub_queries)
        sub_queries_triplet.push_back(getTriplet(q));
    }
    catch(const std::string& msg)
    {
      error_ = msg;
    }

    return sub_queries_triplet;
  }

} // namespace ontologenius
