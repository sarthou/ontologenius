#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/ClassIndexClient.h"

#include <cstdint>
#include <string>
#include <vector>

namespace onto {

  std::vector<int64_t> ClassIndexClient::getDown(int64_t index, int depth, int64_t selector)
  {
    std::string param = std::to_string(index);
    if(depth >= 0)
      param += " -d " + std::to_string(depth);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    return callIndexes("getDown", param);
  }

  std::vector<int64_t> ClassIndexClient::getDisjoint(int64_t index)
  {
    const std::string param = std::to_string(index);
    return callIndexes("getDisjoint", param);
  }

  std::vector<int64_t> ClassIndexClient::getOn(int64_t index, int64_t property, int64_t selector)
  {
    std::string param = std::to_string(index) + ":" + std::to_string(property);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    return callIndexes("getOn", param);
  }

  std::vector<int64_t> ClassIndexClient::getFrom(int64_t property, int64_t index, int64_t selector)
  {
    std::string param = std::to_string(index) + ":" + std::to_string(property);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    return callIndexes("getFrom", param);
  }

  std::vector<int64_t> ClassIndexClient::getWith(int64_t class_from, int64_t class_to, int64_t selector, int depth)
  {
    std::string param = std::to_string(class_from) + ":" + std::to_string(class_to);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getWith", param);
  }

  std::vector<int64_t> ClassIndexClient::getRelatedFrom(int64_t property)
  {
    const std::string param = std::to_string(property);
    return callIndexes("getRelatedFrom", param);
  }

  std::vector<int64_t> ClassIndexClient::getRelatedOn(int64_t property)
  {
    const std::string param = std::to_string(property);
    return callIndexes("getRelatedOn", param);
  }

  std::vector<int64_t> ClassIndexClient::getRelatedWith(int64_t index)
  {
    const std::string param = std::to_string(index);
    return callIndexes("getRelatedWith", param);
  }

  std::vector<int64_t> ClassIndexClient::getRelationFrom(int64_t index, int depth)
  {
    std::string param = std::to_string(index);
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getRelationFrom", param);
  }

  std::vector<int64_t> ClassIndexClient::getRelationOn(int64_t index, int depth)
  {
    std::string param = std::to_string(index);
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getRelationOn", param);
  }

  std::vector<int64_t> ClassIndexClient::getRelationWith(int64_t index)
  {
    const std::string param = std::to_string(index);
    return callIndexes("getRelationWith", param);
  }

  std::vector<int64_t> ClassIndexClient::getDomainOf(int64_t index, int64_t selector, int depth)
  {
    std::string param = std::to_string(index);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getDomainOf", param);
  }

  std::vector<int64_t> ClassIndexClient::getRangeOf(int64_t index, int64_t selector, int depth)
  {
    std::string param = std::to_string(index);
    if(selector != 0)
      param += " -s " + std::to_string(selector);

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return callIndexes("getRangeOf", param);
  }

} // namespace onto