#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/IndividualIndexClient.h"

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
  std::string param = std::to_string(property);
  return callIndexes("getRelatedFrom", param);
}

std::vector<int64_t> IndividualIndexClient::getRelatedOn(int64_t property)
{
  std::string param = std::to_string(property);
  return callIndexes("getRelatedOn", param);
}

std::vector<int64_t> IndividualIndexClient::getRelatedWith(int64_t index)
{
  std::string param = std::to_string(index);
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
  std::string param = std::to_string(index);
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
  std::string param = std::to_string(index);
  return callIndexes("getType", param);
}

std::vector<int64_t> IndividualIndexClient::getSame(int64_t index)
{
  std::string param = std::to_string(index);
  return callIndexes("getSame", param);
}

std::vector<int64_t> IndividualIndexClient::getDistincts(int64_t index)
{
  std::string param = std::to_string(index);
  return callIndexes("getDistincts", param);
}

} // namespace onto