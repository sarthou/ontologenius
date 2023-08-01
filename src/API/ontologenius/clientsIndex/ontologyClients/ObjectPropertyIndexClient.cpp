#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/ObjectPropertyIndexClient.h"

namespace onto {

std::vector<int64_t> ObjectPropertyIndexClient::getDown(int64_t index, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDown";
  srv.request.param = std::to_string(index);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> ObjectPropertyIndexClient::getDisjoint(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDisjoint";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> ObjectPropertyIndexClient::getDomain(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDomain";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> ObjectPropertyIndexClient::getRange(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRange";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> ObjectPropertyIndexClient::getInverse(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getInverse";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

} // namespace onto
