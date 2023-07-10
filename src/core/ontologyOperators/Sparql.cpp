#include "ontologenius/core/ontologyOperators/Sparql.h"

#include "ontologenius/utils/String.h"

namespace ontologenius
{
  Sparql::Sparql() : sparql_pattern_("SELECT\\s*(DISTINCT)?\\s*([^\n]+)([\\s\n]*)WHERE([\\s\n]*)(.*)")
  {
    onto_ = nullptr;
    error_ = "";

    operators_["NOT EXISTS"] = sparql_not_exists;
  }

  std::vector<std::map<std::string, std::string>> Sparql::runStr(const std::string& query)
  {
    return run<std::string>(query);
  }

  std::vector<std::map<std::string, index_t>> Sparql::runIndex(const std::string& query)
  {
    return run<index_t>(query);
  }

  template<typename T>
  std::vector<std::map<std::string, T>> Sparql::run(const std::string& query)
  {
    std::vector<std::map<std::string, T>> res;
    error_ = "";
    if(onto_ == nullptr)
    {
      error_ = "ontology is undefined";
      return res;
    }

    std::smatch match;
    if (std::regex_match(query, match, sparql_pattern_))
    {
      std::string vars = match[2].str();
      removeChar(vars, {'\n', '\r'});
      std::vector<std::string> vars_to_return = split(vars, " ");

      std::string pattern = getPattern(match[5].str());
      removeChar(pattern, {'\n', '\r'});

      auto blocks = getBlocks(pattern);
      if(error_ != "")
        return res;

      for(auto& block : blocks)
      {
        auto triplets = getTriplets<T>(block.raw, ".");
        if(error_ != "")
          return res;

        res = resolve(triplets, block.op, res);
      }
      
      filter(res, vars_to_return, match[1].str() != "");
      return res;
    }
    else
    {
      auto triplets = getTriplets<T>(query, ",");
      if(triplets.size())
        return resolve(triplets);
      else
      {
        Display::error("The query is malformed");
        return {};
      }
    }    
  }

  template<typename T>
  std::vector<std::map<std::string, T>> Sparql::resolve(std::vector<triplet_t<T>> query, SparqlOperator_e op, const std::vector<std::map<std::string, T>>& prev_res)
  {
    std::vector<std::map<std::string, T>> res;
    if(prev_res.size())
    {
      for(auto& prev : prev_res)
      {
        std::vector<std::map<std::string, T>> local_res;
        if(query.size())
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
      std::map<std::string, T> empty_accu;
      return resolve(query, empty_accu);
    }
  }

  template<typename T>
  std::vector<std::map<std::string, T>> Sparql::resolve(const std::vector<triplet_t<T>>& query, const std::map<std::string, T>& accu)
  {
    std::vector<std::map<std::string, T>> res;

    triplet_t<T> current = query[0];

    std::unordered_set<T> values;
    std::string var_name;
    resolveSubQuery(current, accu, var_name, values);

    if(values.size() == 0)
      return res;

    std::vector<triplet_t<T>> new_query(query.begin() + 1, query.end());
    //query.erase(query.begin());

    bool variable_exixts_in_accu = (accu.find(var_name) != accu.end());

    for(auto& value : values)
    {
      std::vector<std::map<std::string, T>> local_res;
      if(new_query.size())
      {
        std::map<std::string, T> new_accu = accu;
        if(variable_exixts_in_accu)
        {
          if(new_accu[var_name] != value)
            continue;
        }
        else
          new_accu[var_name] = value;

        local_res = resolve(new_query, new_accu);
        if(local_res.size() == 0)
          continue;
      }

      if(local_res.size() != 0)
      {
        for(auto& lr : local_res)
        {
          lr[var_name] = value;
          res.push_back(lr);
        }
      }
      else
        res.emplace_back(std::initializer_list<std::pair<const std::string, T>>{{var_name, value}});
    }

    return res;
  }

  template<typename T>
  void Sparql::resolveSubQuery(triplet_t<T> triplet, const std::map<std::string, T>& accu, std::string& var_name, std::unordered_set<T>& values)
  {
    if(triplet.predicat.variable)
      error_ = "predicat can not be a variable in: " + toString(triplet);
    else if(triplet.predicat.name == "isA")
    {
      if(triplet.subject.variable && !triplet.object.variable)
      {
        var_name = triplet.subject.name;
        auto var_it = accu.find(var_name);
        if(var_it != accu.end())
          values = getType(triplet, var_it->second);
        else
          values = getType(triplet, getDefaultSelector<T>());
      }
      else if(!triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.object.name;
        auto var_it = accu.find(var_name);
        if(var_it != accu.end())
          values = getUp(triplet, var_it->second);
        else
          values = getUp(triplet, getDefaultSelector<T>());
      }
      else if(triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.subject.name;
        auto var_it = accu.find(var_name);
        if(var_it != accu.end())
        {
          triplet.subject.value = var_it->second;
          var_name = triplet.object.name;
          var_it = accu.find(var_name);
          if(var_it != accu.end())
            values = getUp(triplet, var_it->second);
          else
            values = getUp(triplet, getDefaultSelector<T>());
        }
        else
        {
          var_it = accu.find(triplet.object.name);
          if(var_it != accu.end())
          {
            triplet.object.value = var_it->second;
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
      if(triplet.subject.variable && !triplet.object.variable)
      {
        var_name = triplet.subject.name;
        auto var_it = accu.find(var_name);
        if(var_it != accu.end())
          values = find(triplet, var_it->second);
        else
          values = find(triplet, getDefaultSelector<T>());
      }
      else if(!triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.object.name;
        auto var_it = accu.find(var_name);
        if(var_it != accu.end())
          values = getName(triplet, var_it->second);
        else
          values = getName(triplet, getDefaultSelector<T>());
      }
      else if(triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.subject.name;
        auto var_it = accu.find(var_name);
        if(var_it != accu.end())
        {
          triplet.subject.name = var_it->second;
          var_name = triplet.object.name;
          var_it = accu.find(var_name);
          if(var_it != accu.end())
            values = getName(triplet, var_it->second);
          else
            values = getName(triplet, getDefaultSelector<T>());
        }
        else
        {
          var_it = accu.find(triplet.object.name);
          if(var_it != accu.end())
          {
            triplet.object.name = var_it->second;
            values = find(triplet, getDefaultSelector<T>());
          }
          else
            error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
        }
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable";
    }*/
    else if(triplet.subject.variable && !triplet.object.variable)
    {
      var_name = triplet.subject.name;
      auto var_it = accu.find(var_name);
      if(var_it != accu.end())
        values = getFrom(triplet, var_it->second);
      else
        values = getFrom(triplet, getDefaultSelector<T>());
    }
    else if(!triplet.subject.variable && triplet.object.variable)
    {
      var_name = triplet.object.name;
      auto var_it = accu.find(var_name);
      if(var_it != accu.end())
        values = getOn(triplet, var_it->second);
      else
        values = getOn(triplet, getDefaultSelector<T>());
    }
    else if(triplet.subject.variable && triplet.object.variable)
    {
      var_name = triplet.subject.name;
      auto var_it = accu.find(var_name);
      if(var_it != accu.end())
      {
        triplet.subject.value = var_it->second;
        var_name = triplet.object.name;
        var_it = accu.find(var_name);
        if(var_it != accu.end())
          values = getOn(triplet, var_it->second);
        else
          values = getOn(triplet, getDefaultSelector<T>());
      }
      else
      {
        var_it = accu.find(triplet.object.name);
        if(var_it != accu.end())
        {
          triplet.object.value = var_it->second;
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
  std::unordered_set<T> Sparql::getOn(const triplet_t<T>& triplet, const T& selector)
  {
    auto res = onto_->individual_graph_.getOn(triplet.subject.value, triplet.predicat.value);
    if(isSelectorDefined(selector) == false)
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<T>({selector});
    else
      return std::unordered_set<T>();
  }

  template<typename T>
  std::unordered_set<T> Sparql::getFrom(const triplet_t<T>& triplet, const T& selector)
  {
    if(isSelectorDefined(selector) == false)
      return onto_->individual_graph_.getFrom(triplet.object.value, triplet.predicat.value);
    else
    {
      // Here we revert the problem as we know what we are expecting for.
      auto res = onto_->individual_graph_.getOn(selector, triplet.predicat.value);
      if(std::find(res.begin(), res.end(), triplet.object.value) != res.end())
        return std::unordered_set<T>({selector});
      else
        return std::unordered_set<T>();
    }
  }

  template<typename T>
  std::unordered_set<T> Sparql::getUp(const triplet_t<T>& triplet, const T& selector)
  {
    if(isSelectorDefined(selector) == false)
      return onto_->individual_graph_.getUp(triplet.subject.value);
    else
    {
      auto is = onto_->individual_graph_.getUp(triplet.subject.value);
      if(std::find(is.begin(), is.end(), selector) != is.end())
        return std::unordered_set<T>({selector});
      else
        return std::unordered_set<T>();
    }
  }

  template<typename T>
  std::unordered_set<T> Sparql::getType(const triplet_t<T>& triplet, const T& selector)
  {
    if(isSelectorDefined(selector) == false)
      return onto_->individual_graph_.getType(triplet.object.value);
    else
    {
      auto types = onto_->individual_graph_.getUp(selector);
      if(std::find(types.begin(), types.end(), triplet.object.value) != types.end())
        return std::unordered_set<T>({selector});
      else
        return std::unordered_set<T>();
    }
  }

  template<typename T>
  std::unordered_set<T> Sparql::find(const triplet_t<T>& triplet, const T& selector)
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
  std::unordered_set<std::string> Sparql::getName(const triplet_t<T>& triplet, const std::string& selector)
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
    size_t begin_of_pattern = text.find("{");
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

  template<typename T>
  std::vector<triplet_t<T>> Sparql::getTriplets(const std::string& query, const std::string& delim)
  {
    std::vector<std::string> sub_queries = split(query, delim);
    std::vector<triplet_t<T>> sub_queries_triplet;
    try
    {
      for(auto& q : sub_queries)
        sub_queries_triplet.push_back(getTriplet<T>(q));
    }
    catch(const std::string& msg)
    {
      error_ = msg;
    }

    return sub_queries_triplet;
  }

  template<typename T> 
  void Sparql::mergeNoOp(const std::map<std::string, T>& base, std::vector<std::map<std::string, T>>& res)
  {
    for(auto& r : res)
    {
      for(auto& var : base)
      {
        if(r.find(var.first) == r.end())
          r.insert(var);
      }
    }
  }

  template<typename T> 
  void Sparql::mergeNotExists(const std::map<std::string, T>& base, std::vector<std::map<std::string, T>>& res)
  {
    if(res.empty())
      res.push_back(base);
    else
      res.clear();
  }

} // namespace ontologenius
