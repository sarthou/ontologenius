#ifndef ONTOLOGENIUS_SPARQLUTILS_H
#define ONTOLOGENIUS_SPARQLUTILS_H

#include <string>
#include <vector>
#include <map>
#include <unordered_map>

#include "ontologenius/utils/String.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius
{

enum SparqlOperator_e
{
  sparql_none,
  sparql_not_exists
};

struct SparqlBlock_t
{
  std::string raw;
  SparqlOperator_e op;
  std::vector<std::map<std::string, std::string>> res;
  std::vector<SparqlBlock_t> sub_blocks;
};

template<typename T>
struct resource_t
{
  resource_t() : variable_id(-1),
                 is_variable(false),
                 regex(false) {}
  
  static std::unordered_map<std::string, int64_t> variables;
  static std::vector<std::string> to_variables;

  std::string name;
  T value;
  int64_t variable_id;
  bool is_variable;
  bool regex;
};

template<typename T>
std::unordered_map<std::string, int64_t> resource_t<T>::variables;

template<typename T>
std::vector<std::string> resource_t<T>::to_variables;

template<typename T>
struct triplet_t
{
  resource_t<T> subject;
  resource_t<T> predicat;
  resource_t<T> object;
};
typedef triplet_t<std::string> strTriplet_t;;

void removeUselessSpace(std::string& text);
void removeChar(std::string& text, const std::vector<char>& delim);

template<typename T> T getDefaultSelector() { return T(); }
template<> index_t getDefaultSelector<index_t>();

template<typename T> bool isSelectorDefined(T value) { return value; }
bool isSelectorDefined(const std::string& value);

template<typename T>
T convertResourceValue(const std::string& value)
{
  return value;
}

template<> index_t convertResourceValue<index_t>(const std::string& value);

template<typename T>
resource_t<T> getResource(std::string resource_txt)
{
  removeUselessSpace(resource_txt);

  resource_t<T> res;

  if(resource_txt[0] == '?')
  {
    res.is_variable = true;
    resource_txt = resource_txt.substr(1);
    auto var_it = resource_t<T>::variables.find(resource_txt);
    if(var_it == resource_t<T>::variables.end())
    {
      int64_t index = resource_t<T>::variables.size();
      resource_t<T>::variables[resource_txt] = index;
      res.variable_id = index;
      resource_t<T>::to_variables.push_back(resource_txt);
    }
    else
      res.variable_id = var_it->second;
  }
  else
  {
    res.is_variable = false;
    if(resource_txt != "isA")
      res.value = convertResourceValue<T>(resource_txt);
  }
  res.name = resource_txt;

  return res;
}

template<typename T>
triplet_t<T> getTriplet(const std::string& triplet_txt)
{
  std::vector<std::string> resources = split(triplet_txt, " ");

  size_t cpt = 0;
  triplet_t<T> res;
  for(const auto& x : resources)
  {
    if(x != "")
    {
      resource_t<T> resource = getResource<T>(x);
      switch (cpt)
      {
        case 0: res.subject = resource; break;
        case 1: res.predicat = resource; break;
        case 2: res.object = resource; break;
        default:
          if(res.object.is_variable == false)
            res.object.name += " " + resource.name;
          else
            throw "invalid triplet format in : " + triplet_txt;
      }
      cpt++;
    }
  }

  return res;
}

template<typename T>
std::string toString(const triplet_t<T>& triplet)
{
  return (triplet.subject.is_variable ? "?" : "") + triplet.subject.name + " " +
          (triplet.predicat.is_variable ? "?" : "") + triplet.predicat.name + " " +
          (triplet.object.is_variable ? "?" : "") + triplet.object.name;
}

template<typename T>
void removeDuplicate(std::vector<std::map<std::string, T>>& vect)
{
  std::sort(vect.begin(), vect.end());
  auto equalLambda = [](const std::map<std::string, T>& lm, const std::map<std::string, T>& rm)
  {
    return lm.size() == rm.size() && std::equal(lm.begin(), lm.end(), rm.begin());
  };

  vect.erase(unique(vect.begin(), vect.end(), equalLambda), vect.end());
}

template<typename T>
std::vector<int64_t> convertVariables(const std::vector<std::string>& var_names)
{
  std::vector<int64_t> res;
  for(auto& var : var_names)
    res.push_back(resource_t<T>::variables[var]);
  std::sort(res.begin(), res.end());
  return res;
}

template<typename T>
void removeDuplicate(std::vector<std::vector<T>>& vect)
{
  vect.erase(unique(vect.begin(), vect.end()), vect.end());
}

template<typename T>
void filter(std::vector<std::map<std::string, T>>& res, const std::vector<std::string>& vars, bool distinct)
{
  if(vars.size())
  {
    if(vars[0] == "*")
      return;

    for(auto& sub_res : res)
    {
      for (auto itr = sub_res.cbegin() ; itr != sub_res.cend() ; )
        itr = (std::find(vars.begin(), vars.end(), "?" + itr->first) == vars.end()) ? sub_res.erase(itr) : std::next(itr);
    }

    if(distinct)
      removeDuplicate(res);
  }
}

template<typename T>
void filter(std::vector<std::vector<T>>& res, const std::vector<std::string>& vars, bool distinct)
{
  if(vars.size())
  {
    if(vars[0] == "*")
      return;

    std::vector<int64_t> var_index = convertVariables<T>(vars);
    
    for(auto& sub_res : res)
    {
      for(auto it = var_index.rbegin(); it != var_index.rend(); ++it)
        sub_res.erase(sub_res.begin() + *it);
    }

    if(distinct)
      removeDuplicate(res);
  }
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_SPARQLUTILS_H