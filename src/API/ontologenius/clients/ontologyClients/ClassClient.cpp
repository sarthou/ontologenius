#include "ontologenius/API/ontologenius/clients/ontologyClients/ClassClient.h"

namespace onto {

  std::vector<std::string> ClassClient::getDown(const std::string& name, int depth, const std::string& selector)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);
    if(selector.empty() == false)
      param += " -s " + selector;

    return call("getDown", param);
  }

  std::vector<std::string> ClassClient::getDisjoint(const std::string& name)
  {
    std::string param = name;
    return call("getDisjoint", param);
  }

  std::vector<std::string> ClassClient::getOn(const std::string& name, const std::string& property, const std::string& selector)
  {
    std::string param = name + ":" + property;
    if(selector.empty() == false)
      param += " -s " + selector;

    return call("getOn", param);
  }

  std::vector<std::string> ClassClient::getFrom(const std::string& property, const std::string& name, const std::string& selector)
  {
    std::string param = name + ":" + property;
    if(selector.empty() == false)
      param += " -s " + selector;

    return call("getFrom", param);
  }

  std::vector<std::string> ClassClient::getWith(const std::string& indiv_1, const std::string& indiv_2, const std::string& selector, int depth)
  {
    std::string param = indiv_1 + ":" + indiv_2;
    if(selector.empty() == false)
      param += " -s " + selector;

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getWith", param);
  }

  std::vector<std::string> ClassClient::getRelatedFrom(const std::string& property)
  {
    std::string param = property;
    return call("getRelatedFrom", param);
  }

  std::vector<std::string> ClassClient::getRelatedOn(const std::string& property)
  {
    std::string param = property;
    return call("getRelatedOn", param);
  }

  std::vector<std::string> ClassClient::getRelatedWith(const std::string& name)
  {
    std::string param = name;
    return call("getRelatedWith", param);
  }

  std::vector<std::string> ClassClient::getRelationFrom(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getRelationFrom", param);
  }

  std::vector<std::string> ClassClient::getRelationOn(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getRelationOn", param);
  }

  std::vector<std::string> ClassClient::getRelationWith(const std::string& name)
  {
    std::string param = name;
    return call("getRelationWith", param);
  }

  std::vector<std::string> ClassClient::getDomainOf(const std::string& name, const std::string& selector, int depth)
  {
    std::string param = name;
    if(selector.empty() == false)
      param += " -s " + selector;

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getDomainOf", param);
  }

  std::vector<std::string> ClassClient::getRangeOf(const std::string& name, const std::string& selector, int depth)
  {
    std::string param = name;
    if(selector.empty() == false)
      param += " -s " + selector;

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getRangeOf", param);
  }

} // namespace onto