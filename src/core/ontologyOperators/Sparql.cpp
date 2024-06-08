#include "ontologenius/core/ontologyOperators/Sparql.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <regex>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h" // for index_t
#include "ontologenius/core/ontologyOperators/SparqlUtils.h"
#include "ontologenius/graphical/Display.h"
#include "ontologenius/utils/String.h"

namespace ontologenius {

  Sparql::Sparql() : onto_(nullptr),
                     sparql_pattern_("SELECT\\s*(DISTINCT)?\\s*([^\n]+)([\\s\n]*)WHERE([\\s\n]*)(.*)")
  {
    operators_["NOT EXISTS"] = sparql_not_exists;
  }

  std::pair<std::vector<std::string>, std::vector<std::vector<std::string>>> Sparql::runStr(const std::string& query, bool single_same)
  {
    single_same_ = single_same;
    return run<std::string>(query);
  }

  std::pair<std::vector<std::string>, std::vector<std::vector<index_t>>> Sparql::runIndex(const std::string& query, bool single_same)
  {
    single_same_ = single_same;
    return run<index_t>(query);
  }

  template<typename T>
  std::pair<std::vector<std::string>, std::vector<std::vector<T>>> Sparql::run(const std::string& query)
  {
    Resource_t<T>::variables.clear();
    Resource_t<T>::to_variables.clear();

    std::vector<std::vector<T>> res;
    error_ = "";
    if(onto_ == nullptr)
    {
      error_ = "ontology is undefined";
      return {{}, {}};
    }

    std::smatch match;
    if(std::regex_match(query, match, sparql_pattern_))
    {
      std::string vars = match[2].str();
      removeChar(vars, {'\n', '\r'});
      std::vector<std::string> vars_to_return = split(vars, " ");

      std::string pattern = getPattern(match[5].str());
      removeChar(pattern, {'\n', '\r'});

      auto blocks = getBlocks(pattern);
      if(error_.empty() == false)
        return {{}, {}};

      for(auto& block : blocks)
      {
        auto triplets = getTriplets<T>(block.raw, ".");
        if(error_.empty() == false)
          return {{}, {}};

        res = resolve(triplets, block.op, res);
      }

      filter(res, vars_to_return, match[1].str().empty() == false);

      if(vars_to_return.empty() == false)
      {
        if(vars_to_return[0] == "*")
          return {Resource_t<T>::to_variables, res};

        const std::vector<int64_t> var_index = convertVariables<T>(vars_to_return);
        std::vector<std::string> ordered_vars;
        ordered_vars.reserve(var_index.size());
        for(auto index : var_index)
          ordered_vars.push_back(Resource_t<T>::to_variables[index]);
        return {ordered_vars, res};
      }
      else
        return {Resource_t<T>::to_variables, std::move(res)};
    }
    else
    {
      auto triplets = getTriplets<T>(query, ",");
      if(triplets.empty() == false)
      {
        auto values = resolve(triplets, std::vector<T>(Resource_t<T>::to_variables.size(), getDefaultSelector<T>()));
        return {Resource_t<T>::to_variables, std::move(values)};
      }
      else
      {
        Display::error("The query is malformed");
        return {{}, {}};
      }
    }
  }

  template<typename T>
  std::vector<std::vector<T>> Sparql::resolve(std::vector<SparqlTriplet_t<T>> query, SparqlOperator_e op, const std::vector<std::vector<T>>& prev_res)
  {
    std::vector<std::vector<T>> res;
    if(prev_res.empty() == false)
    {
      for(auto& prev : prev_res)
      {
        std::vector<std::vector<T>> local_res;
        if(query.empty() == false)
          local_res = resolve(query, prev);

        switch(op)
        {
        case sparql_none: mergeNoOp(prev, local_res); break;
        case sparql_not_exists: mergeNotExists(prev, local_res); break;
        }

        res.insert(res.end(), local_res.begin(), local_res.end());
      }
      return res;
    }
    else
    {
      const std::vector<T> empty_accu(Resource_t<T>::to_variables.size(), getDefaultSelector<T>());
      return resolve(query, empty_accu);
    }
  }

  // accu keep trak of the varaibles already assigned at a given stage
  template<typename T>
  std::vector<std::vector<T>> Sparql::resolve(const std::vector<SparqlTriplet_t<T>>& query, const std::vector<T>& accu)
  {
    std::unordered_set<T> values;
    int64_t var_index = 0;
    resolveSubQuery(query[0], accu, var_index, values);

    if(values.empty())
      return {};

    if(query.size() > 1)
    {
      const std::vector<SparqlTriplet_t<T>> new_query(query.begin() + 1, query.end());

      std::vector<std::vector<T>> res;
      res.reserve(values.size());
      std::vector<T> new_accu(accu);
      for(auto& value : values)
      {
        if(accu[var_index] != getDefaultSelector<T>())
        {
          if(accu[var_index] != value)
            continue;
        }
        else
          new_accu[var_index] = value;

        std::vector<std::vector<T>> local_res = resolve(new_query, new_accu);
        if(local_res.empty() == false)
        {
          for(auto& lr : local_res)
          {
            lr[var_index] = value;
            res.push_back(std::move(lr));
          }
        }
      }
      return res;
    }
    else
    {
      std::vector<std::vector<T>> res(values.size(), std::vector<T>(Resource_t<T>::variables.size(), getDefaultSelector<T>()));
      size_t cpt = 0;
      for(auto& value : values)
        res[cpt++][var_index] = value;
      return res;
    }
  }

  template<typename T>
  void Sparql::resolveSubQuery(SparqlTriplet_t<T> triplet, const std::vector<T>& accu, int64_t& var_index, std::unordered_set<T>& values)
  {
    if(triplet.predicat.is_variable)
      error_ = "predicat can not be a variable in: " + toString(triplet);
    else if(triplet.predicat.name == "isA")
    {
      if(triplet.subject.is_variable && !triplet.object.is_variable)
      {
        var_index = triplet.subject.variable_id;
        values = getType(triplet, accu[var_index]);
      }
      else if(!triplet.subject.is_variable && triplet.object.is_variable)
      {
        var_index = triplet.object.variable_id;
        values = getUp(triplet, accu[var_index]);
      }
      else if(triplet.subject.is_variable && triplet.object.is_variable)
      {
        var_index = triplet.subject.variable_id;
        if(accu[var_index] != getDefaultSelector<T>())
        {
          triplet.subject.value = accu[var_index];
          var_index = triplet.object.variable_id;
          values = getUp(triplet, accu[var_index]);
        }
        else
        {
          if(accu[triplet.object.variable_id] != getDefaultSelector<T>())
          {
            triplet.object.value = accu[triplet.object.variable_id];
            values = getType(triplet, getDefaultSelector<T>());
          }
          else
            error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
        }
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable";
    }
    // The hasLabel has been removed as it does not work with indexes
    /*else if(triplet.predicat.name == "hasLabel")
    {
      if(triplet.subject.is_variable && !triplet.object.is_variable)
      {
        var_index = triplet.subject.variable_id;
        values = find(triplet, accu[var_index]);
      }
      else if(!triplet.subject.is_variable && triplet.object.is_variable)
      {
        var_index = triplet.object.variable_id;
        values = getName(triplet, accu[var_index]);
      }
      else if(triplet.subject.is_variable && triplet.object.is_variable)
      {
        var_index = triplet.subject.variable_id;
        if(accu[var_index] != getDefaultSelector<T>())
        {
          triplet.subject.name = accu[var_index];
          var_index = triplet.object.variable_id;
          values = getName(triplet, accu[var_index]);
        }
        else
        {
          if(accu[triplet.object.name] != getDefaultSelector<T>())
          {
            triplet.object.name = accu[triplet.object.name];
            values = find(triplet, getDefaultSelector<T>());
          }
          else
            error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
        }
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable";
    }*/
    else if(triplet.subject.is_variable && !triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      values = getFrom(triplet, accu[var_index]);
    }
    else if(!triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.object.variable_id;
      values = getOn(triplet, accu[var_index]);
    }
    else if(triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      if(accu[var_index] != getDefaultSelector<T>())
      {
        triplet.subject.value = accu[var_index];
        var_index = triplet.object.variable_id;
        values = getOn(triplet, accu[var_index]);
      }
      else
      {
        if(accu[triplet.object.variable_id] != getDefaultSelector<T>())
        {
          triplet.object.value = accu[triplet.object.variable_id];
          values = getFrom(triplet, getDefaultSelector<T>());
        }
        else
          error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
      }
    }
    else
      error_ = "can not resolve query : " + toString(triplet) + " : No variable";
  }

  template<typename T>
  std::unordered_set<T> Sparql::getOn(const SparqlTriplet_t<T>& triplet, const T& selector)
  {
    auto res = onto_->individual_graph_.getOn(triplet.subject.value, triplet.predicat.value, single_same_);
    if(isSelectorDefined(selector) == false)
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<T>({selector});
    else
      return std::unordered_set<T>();
  }

  template<typename T>
  std::unordered_set<T> Sparql::getFrom(const SparqlTriplet_t<T>& triplet, const T& selector)
  {
    if(isSelectorDefined(selector) == false)
      return onto_->individual_graph_.getFrom(triplet.object.value, triplet.predicat.value, single_same_);
    else
    {
      // Here we revert the problem as we know what we are expecting for.
      auto res = onto_->individual_graph_.getOn(selector, triplet.predicat.value, single_same_);
      if(std::find(res.begin(), res.end(), triplet.object.value) != res.end())
        return std::unordered_set<T>({selector});
      else
        return std::unordered_set<T>();
    }
  }

  template<typename T>
  std::unordered_set<T> Sparql::getUp(const SparqlTriplet_t<T>& triplet, const T& selector)
  {
    if(isSelectorDefined(selector) == false)
      return onto_->individual_graph_.getUp(triplet.subject.value);
    else
    {
      // auto is = onto_->individual_graph_.getUp(triplet.subject.value);
      // if(std::find(is.begin(), is.end(), selector) != is.end())
      if(onto_->individual_graph_.isA(triplet.subject.value, selector))
        return std::unordered_set<T>({selector});
      else
        return std::unordered_set<T>();
    }
  }

  template<typename T>
  std::unordered_set<T> Sparql::getType(const SparqlTriplet_t<T>& triplet, const T& selector)
  {
    if(isSelectorDefined(selector) == false)
      return onto_->individual_graph_.getType(triplet.object.value, single_same_);
    else
    {
      // auto types = onto_->individual_graph_.getUp(selector);
      // if(std::find(types.begin(), types.end(), triplet.object.value) != types.end())
      if(onto_->individual_graph_.isA(selector, triplet.object.value))
        return {selector};
      else
        return std::unordered_set<T>();
    }
  }

  template<typename T>
  std::unordered_set<T> Sparql::find(const SparqlTriplet_t<T>& triplet, const T& selector)
  {
    auto res = onto_->individual_graph_.find<T>(triplet.object.name);
    if(isSelectorDefined(selector) == false)
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<T>({selector});
    else
      return std::unordered_set<T>();
  }

  template<typename T>
  std::unordered_set<std::string> Sparql::getName(const SparqlTriplet_t<T>& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.getNames(triplet.subject.value);
    if(isSelectorDefined(selector) == false)
    {
      std::unordered_set<std::string> set_res;
      for(auto& r : res)
        set_res.insert(r);
      return set_res;
    }
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<T>();
  }

  std::string Sparql::getPattern(const std::string& text)
  {
    const size_t begin_of_pattern = text.find('{');
    std::string res;
    getIn(begin_of_pattern, res, text, '{', '}');
    return res;
  }

  std::vector<SparqlBlock_t> Sparql::getBlocks(std::string query)
  {
    removeUselessSpace(query);

    std::vector<SparqlBlock_t> res;
    std::map<std::string, std::string> tmp_blocks;
    std::set<std::string> blocks_id;
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
      tmp_blocks[mark] = text_in;
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

  template<typename T>
  std::vector<SparqlTriplet_t<T>> Sparql::getTriplets(const std::string& query, const std::string& delim)
  {
    const std::vector<std::string> sub_queries = split(query, delim);
    std::vector<SparqlTriplet_t<T>> sub_queries_triplet;
    try
    {
      for(const auto& q : sub_queries)
        sub_queries_triplet.push_back(getTriplet<T>(q));
    }
    catch(const std::string& msg)
    {
      error_ = msg;
    }

    return sub_queries_triplet;
  }

  template<typename T>
  void Sparql::mergeNoOp(const std::vector<T>& base, std::vector<std::vector<T>>& res)
  {
    for(auto& r : res)
    {
      for(size_t i = 0; i < base.size(); i++)
      {
        if(r[i] == getDefaultSelector<T>())
          r[i] = base[i];
      }
    }
  }

  template<typename T>
  void Sparql::mergeNotExists(const std::vector<T>& base, std::vector<std::vector<T>>& res)
  {
    if(res.empty())
      res.push_back(base);
    else
      res.clear();
  }

} // namespace ontologenius
