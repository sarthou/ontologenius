#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/DataPropertyIndexClient.h"

namespace onto {

std::vector<int64_t> DataPropertyIndexClient::getDown(int64_t index, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDown";
  srv.request.param = std::to_string(index);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> DataPropertyIndexClient::getDisjoint(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDisjoint";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> DataPropertyIndexClient::getDomain(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDomain";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> DataPropertyIndexClient::getRange(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRange";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

} // namespace onto
