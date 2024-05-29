#include "ontologenius/API/ontologenius/clients/ontologyClients/IndividualClient.h"

namespace onto {

  std::vector<std::string> IndividualClient::getOn(const std::string& name, const std::string& property, const std::string& selector)
  {
    std::string param = name + ":" + property;
    if(selector.empty() == false)
      param += " -s " + selector;

    return call("getOn", param);
  }

  std::vector<std::string> IndividualClient::getFrom(const std::string& property, const std::string& name, const std::string& selector)
  {
    std::string param = name + ":" + property;
    if(selector.empty() == false)
      param += " -s " + selector;

    return call("getFrom", param);
  }

  std::vector<std::string> IndividualClient::getWith(const std::string& indiv_from, const std::string& indiv_to, const std::string& selector, int depth)
  {
    std::string param = indiv_from + ":" + indiv_to;
    if(selector.empty() == false)
      param += " -s " + selector;

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getWith", param);
  }

  std::vector<std::string> IndividualClient::getRelatedFrom(const std::string& property)
  {
    std::string param = property;
    return call("getRelatedFrom", param);
  }

  std::vector<std::string> IndividualClient::getRelatedOn(const std::string& property)
  {
    std::string param = property;
    return call("getRelatedOn", param);
  }

  std::vector<std::string> IndividualClient::getRelatedWith(const std::string& name)
  {
    std::string param = name;
    return call("getRelatedWith", param);
  }

  std::vector<std::string> IndividualClient::getRelationFrom(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getRelationFrom", param);
  }

  std::vector<std::string> IndividualClient::getRelationOn(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getRelationOn", param);
  }

  std::vector<std::string> IndividualClient::getRelationWith(const std::string& name)
  {
    std::string param = name;
    return call("getRelationWith", param);
  }

  std::vector<std::string> IndividualClient::getDomainOf(const std::string& name, const std::string& selector, int depth)
  {
    std::string param = name;
    if(selector.empty() == false)
      param += " -s " + selector;

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getDomainOf", param);
  }

  std::vector<std::string> IndividualClient::getRangeOf(const std::string& name, const std::string& selector, int depth)
  {
    std::string param = name;
    if(selector.empty() == false)
      param += " -s " + selector;

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getRangeOf", param);
  }

  std::vector<std::string> IndividualClient::getType(const std::string& name)
  {
    std::string param = name;
    return call("getType", param);
  }

  std::vector<std::string> IndividualClient::getSame(const std::string& name)
  {
    std::string param = name;
    return call("getSame", param);
  }

  std::vector<std::string> IndividualClient::getDistincts(const std::string& name)
  {
    std::string param = name;
    return call("getDistincts", param);
  }

} // namespace onto