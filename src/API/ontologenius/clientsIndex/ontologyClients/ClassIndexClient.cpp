#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/ClassIndexClient.h"

namespace onto {

std::vector<int64_t> ClassIndexClient::getDown(int64_t index, int depth, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDown";
  srv.request.param = std::to_string(index);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getDisjoint(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDisjoint";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getOn(int64_t index, int64_t property, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getOn";
  srv.request.param = std::to_string(index) + ":" + std::to_string(property);
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getFrom(int64_t property, int64_t index, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getFrom";
  srv.request.param = std::to_string(index) + ":" + std::to_string(property);
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getWith(int64_t indiv_1, int64_t indiv_2, int64_t selector, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getWith";
  srv.request.param = std::to_string(indiv_1) + ":" + std::to_string(indiv_2);
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getRelatedFrom(int64_t property)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelatedFrom";
  srv.request.param = std::to_string(property);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getRelatedOn(int64_t property)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelatedOn";
  srv.request.param = std::to_string(property);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getRelatedWith(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelatedWith";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getRelationFrom(int64_t index, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelationFrom";
  srv.request.param = std::to_string(index);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getRelationOn(int64_t index, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelationOn";
  srv.request.param = std::to_string(index);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getRelationWith(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelationWith";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getDomainOf(int64_t index, int64_t selector, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDomainOf";
  srv.request.param = std::to_string(index);

  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> ClassIndexClient::getRangeOf(int64_t index, int64_t selector, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRangeOf";
  srv.request.param = std::to_string(index);

  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

} // namespace onto