#include "ontologenius/API/ontologenius/clients/ontologyClients/IndividualClient.h"

#include <string>
#include <vector>

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
    return call("getRelatedFrom", property);
  }

  std::vector<std::string> IndividualClient::getRelatedOn(const std::string& property)
  {
    return call("getRelatedOn", property);
  }

  std::vector<std::string> IndividualClient::getRelatedWith(const std::string& name)
  {
    return call("getRelatedWith", name);
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
    return call("getRelationWith", name);
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
    return call("getType", name);
  }

  std::vector<std::string> IndividualClient::getSame(const std::string& name)
  {
    return call("getSame", name);
  }

  std::vector<std::string> IndividualClient::getDistincts(const std::string& name)
  {
    return call("getDistincts", name);
  }

  bool IndividualClient::isInferred(const std::string& subject, const std::string& property, const std::string& object)
  {
    std::string param = subject + "|" + property + "|" + object;
    return (callStr("isInferred", param).empty() == false);
  }

  bool IndividualClient::isInferred(const std::string& subject, const std::string& class_name)
  {
    std::string param = subject + "|" + class_name;
    return (callStr("isInferred", param).empty() == false);
  }

  std::vector<std::string> IndividualClient::getInferenceExplanation(const std::string& subject, const std::string& property, const std::string& object)
  {
    std::string param = subject + "|" + property + "|" + object;
    return call("getInferenceExplanation", param);
  }

  std::vector<std::string> IndividualClient::getInferenceExplanation(const std::string& subject, const std::string& class_name)
  {
    std::string param = subject + "|" + class_name;
    return call("getInferenceExplanation", param);
  }

} // namespace onto