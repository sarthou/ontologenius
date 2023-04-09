#include "ontologenius/core/ontologyOperators/SparqlUtils.h"

#include "ontologenius/utils/String.h"

#include <algorithm>

namespace ontologenius
{

  triplet_t getTriplet(const std::string& triplet_txt)
  {
    std::vector<std::string> resources = split(triplet_txt, " ");

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
          default:
            if(res.object.variable == false)
              res.object.name += " " + resource.name;
            else
              throw "invalid triplet format in : " + triplet_txt;
        }
        cpt++;
      }
    }

    return res;
  }

  resource_t getResource(std::string resource_txt)
  {
    removeUselessSpace(resource_txt);

    resource_t res;

    if(resource_txt[0] == '?')
    {
      res.variable = true;
      resource_txt = resource_txt.substr(1);
    }
    else
      res.variable = false;
    res.name = resource_txt;

    return res;
  }

  std::string toString(const triplet_t& triplet)
  {
    return (triplet.subject.variable ? "?" : "") + triplet.subject.name + " " +
            (triplet.predicat.variable ? "?" : "") + triplet.predicat.name + " " +
            (triplet.object.variable ? "?" : "") + triplet.object.name;
  }

  void removeUselessSpace(std::string& text)
  {
    while((text[0] == ' ') && (text.size() != 0))
      text.erase(0,1);

    while((text[text.size() - 1] == ' ') && (text.size() != 0))
      text.erase(text.size() - 1,1);
  }

  void removeChar(std::string& text, const std::vector<char>& delim)
  {
    for(auto c : delim)
      text.erase(std::remove(text.begin(), text.end(), c), text.end());
  }

  void filter(std::vector<std::map<std::string, std::string>>& res, const std::vector<std::string>& vars, bool distinct)
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

  void removeDuplicate(std::vector<std::map<std::string, std::string>>& vect)
  {
    std::sort(vect.begin(), vect.end());
    auto equalLambda = [](const std::map<std::string, std::string>& lm, const std::map<std::string, std::string>& rm)
    {
      return lm.size() == rm.size() && std::equal(lm.begin(), lm.end(), rm.begin());
    };

    vect.erase(unique(vect.begin(), vect.end(), equalLambda), vect.end());
  }

} // namespace ontologenius