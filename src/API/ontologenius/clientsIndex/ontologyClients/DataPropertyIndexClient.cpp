#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/DataPropertyIndexClient.h"

namespace onto {

std::vector<int64_t> DataPropertyIndexClient::getDown(int64_t index, int depth)
{
  std::string param = std::to_string(index);
  if(depth >= 0)
    param += " -d " + std::to_string(depth);

  return callIndexes("getDown", param);
}

std::vector<int64_t> DataPropertyIndexClient::getDisjoint(int64_t index)
{
  std::string param = std::to_string(index);
  return callIndexes("getDisjoint", param);
}

std::vector<int64_t> DataPropertyIndexClient::getDomain(int64_t index, int depth)
{
  std::string param = std::to_string(index);
  if(depth >= 0)
    param += " -d " + std::to_string(depth);

  return callIndexes("getDomain", param);
}

std::vector<int64_t> DataPropertyIndexClient::getRange(int64_t index)
{
  std::string param = std::to_string(index);
  return callIndexes("getRange", param);
}

} // namespace onto
