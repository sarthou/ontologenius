#include "ontologenius/core/ontologyOperators/Sparql.h"

#include "ontologenius/utils/String.h"

namespace ontologenius
{
  Sparql::Sparql() : sparql_pattern_("SELECT\\s*(DISTINCT)?\\s*([^\n]+)([\\s\n]*)WHERE([\\s\n]*)\\{([^\\}]*)\\}")
  {
    onto_ = nullptr;
    error_ = "";
  }

  std::vector<std::map<std::string, std::string>> Sparql::run(const std::string& query)
  {
    if(onto_ == nullptr)
    {
      error_ = "ontology is undefined";
      return std::vector<std::map<std::string, std::string>>();
    }

    std::smatch match;
    if (std::regex_match(query, match, sparql_pattern_))
    {
      std::vector<std::string> vars_to_return = split(match[2].str(), " ");
      auto res = resolve(getTriplets(match[5].str(), "."));
      filter(res, vars_to_return, match[1].str() != "");
      return res;
    }
    else
    {
      return resolve(getTriplets(query, ","));
    }    
  }

  std::vector<std::map<std::string, std::string>> Sparql::resolve(std::vector<triplet_t> query, const std::map<std::string, std::string>& accu)
  {
    std::vector<std::map<std::string, std::string>> res;

    triplet_t current = query[0];
    query.erase(query.begin());

    std::unordered_set<std::string> values;
    std::string var_name;
    resolveSubQuery(current, accu, var_name, values);

    for(auto& value : values)
    {
      std::map<std::string, std::string> new_accu = accu;
      if(new_accu.find(var_name) != new_accu.end())
      {
        if(new_accu[var_name] != value)
          continue;
      }
      else
        new_accu[var_name] = value;


      std::vector<std::map<std::string, std::string>> local_res;
      if(query.size())
      {
        local_res = resolve(query, new_accu);
        if(local_res.size() == 0)
          continue;
      }

      if(local_res.size() != 0)
      {
        for(auto lr : local_res)
        {
          lr[var_name] = value;
          res.push_back(lr);
        }
      }
      else
      {
        std::map<std::string, std::string> tmp;
        tmp[var_name] = value;
        res.push_back(tmp);
      }
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
          error_ = "can not resolve query : " + toString(triplet);
      }
      else
        error_ = "can not resolve query : " + toString(triplet);
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
          error_ = "can not resolve query : " + toString(triplet);
      }
      else
        error_ = "can not resolve query : " + toString(triplet);
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
        error_ = "can not resolve query : " + toString(triplet);
    }
    else
      error_ = "can not resolve query : " + toString(triplet);
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
    auto res = onto_->individual_graph_.getFrom(triplet.object.name, triplet.predicat.name);
    if(selector == "")
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
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
    auto res = onto_->individual_graph_.getType(triplet.object.name);
    if(selector == "")
      return res;
    else if(std::find(res.begin(), res.end(), selector) != res.end())
      return std::unordered_set<std::string>({selector});
    else
      return std::unordered_set<std::string>();
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

  std::vector<triplet_t> Sparql::getTriplets(const std::string& query, const std::string& delim)
  {
    std::vector<std::string> sub_queries = split(query, delim);
    std::vector<triplet_t> sub_queries_triplet;
    for(auto& q : sub_queries)
      sub_queries_triplet.push_back(getTriplet(q));

    return sub_queries_triplet;
  }

  triplet_t Sparql::getTriplet(const std::string& subquery)
  {
    std::vector<std::string> resources = split(subquery, " ");

    size_t cpt = 0;
    triplet_t res;
    for(const auto& x : resources)
    {
      if(x != "")
      {
        resource_t resource = getResource(x);
        switch (cpt)
        {
          case 0: res.subject = resource; break;
          case 1: res.predicat = resource; break;
          case 2: res.object = resource; break;
          default: error_ = "invalid subquery format in : " + subquery;
        }
        cpt++;
      }
    }

    return res;
  }

  resource_t Sparql::getResource(const std::string& resource)
  {
    std::string text = resource;
    removeUselessSpace(text);

    resource_t res;

    if(text[0] == '?')
    {
      res.variable = true;
      text = text.substr(1);
    }
    else
      res.variable = false;
    res.name = text;

    return res;
  }

  std::string Sparql::toString(const triplet_t& triplet)
  {
    return (triplet.subject.variable ? "?" : "") + triplet.subject.name + " " +
            (triplet.predicat.variable ? "?" : "") + triplet.predicat.name + " " +
            (triplet.object.variable ? "?" : "") + triplet.object.name;
  }

  void Sparql::removeUselessSpace(std::string& text)
  {
    while((text[0] == ' ') && (text.size() != 0))
      text.erase(0,1);

    while((text[text.size() - 1] == ' ') && (text.size() != 0))
      text.erase(text.size() - 1,1);
  }

  void Sparql::filter(std::vector<std::map<std::string, std::string>>& res, const std::vector<std::string>& vars, bool distinct)
  {
    if(vars.size())
    {
      if(vars[0] == "*")
        return;

      for(auto& sub_res : res)
      {
        for (std::map<std::string, std::string>::const_iterator itr = sub_res.cbegin() ; itr != sub_res.cend() ; )
          itr = (std::find(vars.begin(), vars.end(), "?" + itr->first) == vars.end()) ? sub_res.erase(itr) : std::next(itr);
      }

      if(distinct)
        removeDuplicate(res);
    }
  }

  void Sparql::removeDuplicate(std::vector<std::map<std::string, std::string>>& vect)
  {
    std::sort(vect.begin(), vect.end());
    auto equalLambda = [](const std::map<std::string, std::string>& lm, const std::map<std::string, std::string>& rm)
    {
      return lm.size() == rm.size() && std::equal(lm.begin(), lm.end(), rm.begin());
    };

    vect.erase(unique(vect.begin(), vect.end(), equalLambda), vect.end());
  }

} // namespace ontologenius
