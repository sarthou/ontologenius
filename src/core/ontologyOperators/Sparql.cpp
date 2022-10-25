#include "ontologenius/core/ontologyOperators/Sparql.h"

#include "ontologenius/utils/String.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius
{
  Sparql::Sparql() : sparql_pattern_("SELECT\\s*(DISTINCT)?\\s*([^\n]+)([\\s\n]*)WHERE([\\s\n]*)(.*)")
  {
    onto_ = nullptr;
    error_ = "";

    operators_["NOT EXISTS"] = sparql_not_exists;
  }

  std::vector<std::map<std::string, std::string>> Sparql::run(const std::string& query)
  {
    std::vector<std::map<std::string, std::string>> res;
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
        auto triplets = getTriplets(block.raw, ".");
        if(error_ != "")
          return res;

        res = resolve(triplets, block.op, res);
      }
      
      filter(res, vars_to_return, match[1].str() != "");
      return res;
    }
    else
    {
      auto triplets = getTriplets(query, ",");
      if(triplets.size())
        return resolve(triplets);
      else
      {
        Display::error("The query is malformed");
        return {};
      }
    }    
  }

  std::vector<std::map<std::string, std::string>> Sparql::resolve(std::vector<triplet_t> query, SparqlOperator_e op, const std::vector<std::map<std::string, std::string>>& prev_res)
  {
    std::vector<std::map<std::string, std::string>> res;
    if(prev_res.size())
    {
      for(auto& prev : prev_res)
      {
        std::vector<std::map<std::string, std::string>> local_res;
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
      std::map<std::string, std::string> empty_accu;
      return resolve(query, empty_accu);
    }
  }

  std::vector<std::map<std::string, std::string>> Sparql::resolve(const std::vector<triplet_t>& query, const std::map<std::string, std::string>& accu)
  {
    std::vector<std::map<std::string, std::string>> res;

    triplet_t current = query[0];

    std::unordered_set<std::string> values;
    std::string var_name;
    resolveSubQuery(current, accu, var_name, values);

    if(values.size() == 0)
      return res;

    std::vector<triplet_t> new_query(query.begin() + 1, query.end());
    //query.erase(query.begin());

    bool variable_exixts_in_accu = (accu.find(var_name) != accu.end());

    for(auto& value : values)
    {
      std::vector<std::map<std::string, std::string>> local_res;
      if(new_query.size())
      {
        std::map<std::string, std::string> new_accu = accu;
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
        res.emplace_back(std::initializer_list<std::pair<const std::string, std::string>>{{var_name, value}});
    }

    return res;
  }

  void Sparql::resolveSubQuery(triplet_t triplet, const std::map<std::string, std::string>& accu, std::string& var_name, std::unordered_set<std::string>& values)
  {
    if(triplet.predicat.variable)
      error_ = "predicat can not be a variable in: " + toString(triplet);
    else if(triplet.predicat.name == "isA")
    {
      if(triplet.subject.variable && !triplet.object.variable)
      {
        var_name = triplet.subject.name;
        if(accu.find(var_name) != accu.end())
          values = getType(triplet, accu.at(var_name));
        else
          values = getType(triplet);
      }
      else if(!triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.object.name;
        if(accu.find(var_name) != accu.end())
          values = getUp(triplet, accu.at(var_name));
        else
          values = getUp(triplet);
      }
      else if(triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.subject.name;
        if(accu.find(var_name) != accu.end())
        {
          triplet.subject.name = accu.at(triplet.subject.name);
          var_name = triplet.object.name;
          if(accu.find(var_name) != accu.end())
            values = getUp(triplet, accu.at(var_name));
          else
            values = getUp(triplet);
        }
        else if(accu.find(triplet.object.name) != accu.end())
        {
          triplet.object.name = accu.at(triplet.object.name);
          values = getType(triplet);
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
        if(accu.find(var_name) != accu.end())
          values = find(triplet, accu.at(var_name));
        else
          values = find(triplet);
      }
      else if(!triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.object.name;
        if(accu.find(var_name) != accu.end())
          values = getName(triplet, accu.at(var_name));
        else
          values = getName(triplet);
      }
      else if(triplet.subject.variable && triplet.object.variable)
      {
        var_name = triplet.subject.name;
        if(accu.find(var_name) != accu.end())
        {
          triplet.subject.name = accu.at(triplet.subject.name);
          var_name = triplet.object.name;
          if(accu.find(var_name) != accu.end())
            values = getName(triplet, accu.at(var_name));
          else
            values = getName(triplet);
        }
        else if(accu.find(triplet.object.name) != accu.end())
        {
          triplet.object.name = accu.at(triplet.object.name);
          values = find(triplet);
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
      if(accu.find(var_name) != accu.end())
        values = getFrom(triplet, accu.at(var_name));
      else
        values = getFrom(triplet);
    }
    else if(!triplet.subject.variable && triplet.object.variable)
    {
      var_name = triplet.object.name;
      if(accu.find(var_name) != accu.end())
        values = getOn(triplet, accu.at(var_name));
      else
        values = getOn(triplet);
    }
    else if(triplet.subject.variable && triplet.object.variable)
    {
      var_name = triplet.subject.name;
      if(accu.find(var_name) != accu.end())
      {
        triplet.subject.name = accu.at(triplet.subject.name);
        var_name = triplet.object.name;
        if(accu.find(var_name) != accu.end())
          values = getOn(triplet, accu.at(var_name));
        else
          values = getOn(triplet);
      }
      else if(accu.find(triplet.object.name) != accu.end())
      {
        triplet.object.name = accu.at(triplet.object.name);
        values = getFrom(triplet);
      }
      else
        error_ = "can not resolve query : " + toString(triplet) + " : No variable already bounded";
    }
    else
      error_ = "can not resolve query : " + toString(triplet) + " : No variable";
  }

  std::unordered_set<std::string> Sparql::getOn(const triplet_t& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.getOn(triplet.subject.name, triplet.predicat.name);
    if(selector == "")
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
  }

  std::unordered_set<std::string> Sparql::getFrom(const triplet_t& triplet, const std::string& selector)
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

  std::unordered_set<std::string> Sparql::getUp(const triplet_t& triplet, const std::string& selector)
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

  std::unordered_set<std::string> Sparql::getType(const triplet_t& triplet, const std::string& selector)
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

  std::unordered_set<std::string> Sparql::find(const triplet_t& triplet, const std::string& selector)
  {
    auto res = onto_->individual_graph_.find(triplet.object.name);
    if(selector == "")
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
  }

  std::unordered_set<std::string> Sparql::getName(const triplet_t& triplet, const std::string& selector)
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

  std::vector<triplet_t> Sparql::getTriplets(const std::string& query, const std::string& delim)
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

  void Sparql::mergeNoOp(const std::map<std::string, std::string>& base, std::vector<std::map<std::string, std::string>>& res)
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

  void Sparql::mergeNotExists(const std::map<std::string, std::string>& base, std::vector<std::map<std::string, std::string>>& res)
  {
    if(res.empty())
      res.push_back(base);
    else
      res.clear();
  }

} // namespace ontologenius
