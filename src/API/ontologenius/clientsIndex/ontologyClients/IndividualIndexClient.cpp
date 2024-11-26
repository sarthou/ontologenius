#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/IndividualIndexClient.h"

#include <cstdint>
#include <string>
#include <vector>

namespace onto {

  std::vector<int64_t> IndividualIndexClient::getOn(int64_t index, int64_t property, int64_t selector)
  {
    std::string param = std::to_string(index) + ":" + std::to_string(property);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    return callIndexes("getOn", param);
  }

  std::vector<int64_t> IndividualIndexClient::getFrom(int64_t property, int64_t index, int64_t selector)
  {
    std::string param = std::to_string(index) + ":" + std::to_string(property);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    return callIndexes("getFrom", param);
  }

  std::vector<int64_t> IndividualIndexClient::getWith(int64_t indiv_from, int64_t indiv_to, int64_t selector, int depth)
  {
    std::string param = std::to_string(indiv_from) + ":" + std::to_string(indiv_to);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getWith", param);
  }

  std::vector<int64_t> IndividualIndexClient::getRelatedFrom(int64_t property)
  {
    const std::string param = std::to_string(property);
    return callIndexes("getRelatedFrom", param);
  }

  std::vector<int64_t> IndividualIndexClient::getRelatedOn(int64_t property)
  {
    const std::string param = std::to_string(property);
    return callIndexes("getRelatedOn", param);
  }

  std::vector<int64_t> IndividualIndexClient::getRelatedWith(int64_t index)
  {
    const std::string param = std::to_string(index);
    return callIndexes("getRelatedWith", param);
  }

  std::vector<int64_t> IndividualIndexClient::getRelationFrom(int64_t index, int depth)
  {
    std::string param = std::to_string(index);
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getRelationFrom", param);
  }

  std::vector<int64_t> IndividualIndexClient::getRelationOn(int64_t index, int depth)
  {
    std::string param = std::to_string(index);
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getRelationOn", param);
  }

  std::vector<int64_t> IndividualIndexClient::getRelationWith(int64_t index)
  {
    const std::string param = std::to_string(index);
    return callIndexes("getRelationWith", param);
  }

  std::vector<int64_t> IndividualIndexClient::getDomainOf(int64_t index, int64_t selector, int depth)
  {
    std::string param = std::to_string(index);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getDomainOf", param);
  }

  std::vector<int64_t> IndividualIndexClient::getRangeOf(int64_t index, int64_t selector, int depth)
  {
    std::string param = std::to_string(index);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getRangeOf", param);
  }

  std::vector<int64_t> IndividualIndexClient::getType(int64_t index)
  {
    const std::string param = std::to_string(index);
    return callIndexes("getType", param);
  }

  std::vector<int64_t> IndividualIndexClient::getSame(int64_t index)
  {
    const std::string param = std::to_string(index);
    return callIndexes("getSame", param);
  }

  std::vector<int64_t> IndividualIndexClient::getDistincts(int64_t index)
  {
    const std::string param = std::to_string(index);
    return callIndexes("getDistincts", param);
  }

  bool IndividualIndexClient::isInferred(int64_t subject, int64_t property, int64_t object)
  {
    std::string param = std::to_string(subject) + "|" + std::to_string(property) + "|" + std::to_string(object);
    return (callStr("isInferred", param).empty() == false);
  }

  bool IndividualIndexClient::isInferred(int64_t subject, int64_t class_index)
  {
    std::string param = std::to_string(subject) + "|" + std::to_string(class_index);
    return (callStr("isInferred", param).empty() == false);
  }

  std::vector<std::string> IndividualIndexClient::getInferenceExplanation(int64_t subject, int64_t property, int64_t object)
  {
    std::string param = std::to_string(subject) + "|" + std::to_string(property) + "|" + std::to_string(object);
    return call("getInferenceExplanation", param);
  }

  std::vector<std::string> IndividualIndexClient::getInferenceExplanation(int64_t subject, int64_t class_index)
  {
    std::string param = std::to_string(subject) + "|" + std::to_string(class_index);
    return call("getInferenceExplanation", param);
  }

} // namespace onto