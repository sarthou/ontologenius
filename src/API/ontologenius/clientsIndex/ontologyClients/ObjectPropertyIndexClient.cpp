#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/ObjectPropertyIndexClient.h"

namespace onto {

std::vector<int64_t> ObjectPropertyIndexClient::getDown(int64_t index, int depth)
{
  std::string param = std::to_string(index);
  if(depth >= 0)
    param += " -d " + std::to_string(depth);

  return callIndexes("getDown", param);
}

std::vector<int64_t> ObjectPropertyIndexClient::getDisjoint(int64_t index)
{
  std::string param = std::to_string(index);
  return callIndexes("getDisjoint", param);
}

std::vector<int64_t> ObjectPropertyIndexClient::getDomain(int64_t index, int depth)
{
  std::string param = std::to_string(index);
  if(depth >= 0)
    param += " -d " + std::to_string(depth);

  return callIndexes("getDomain", param);
}

std::vector<int64_t> ObjectPropertyIndexClient::getRange(int64_t index, int depth)
{
  std::string param = std::to_string(index);
  if(depth >= 0)
    param += " -d " + std::to_string(depth);

  return callIndexes("getRange", param);
}

std::vector<int64_t> ObjectPropertyIndexClient::getInverse(int64_t index)
{
  std::string param = std::to_string(index);
  return callIndexes("getInverse", param);
}

} // namespace onto
