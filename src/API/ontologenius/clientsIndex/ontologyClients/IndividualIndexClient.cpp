#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/IndividualIndexClient.h"

namespace onto {

std::vector<int64_t> IndividualIndexClient::getOn(int64_t index, int64_t property, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getOn";
  srv.request.param = std::to_string(index) + ":" + std::to_string(property);
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getFrom(int64_t property, int64_t index, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getFrom";
  srv.request.param = std::to_string(index) + ":" + std::to_string(property);
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getWith(int64_t indiv_from, int64_t indiv_to, int64_t selector, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getWith";
  srv.request.param = std::to_string(indiv_from) + ":" + std::to_string(indiv_to);
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getRelatedFrom(int64_t property)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelatedFrom";
  srv.request.param = std::to_string(property);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getRelatedOn(int64_t property)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelatedOn";
  srv.request.param = std::to_string(property);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getRelatedWith(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelatedWith";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getRelationFrom(int64_t index, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelationFrom";
  srv.request.param = std::to_string(index);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getRelationOn(int64_t index, int depth)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelationOn";
  srv.request.param = std::to_string(index);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getRelationWith(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getRelationWith";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getDomainOf(int64_t index, int64_t selector, int depth)
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

std::vector<int64_t> IndividualIndexClient::getRangeOf(int64_t index, int64_t selector, int depth)
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

std::vector<int64_t> IndividualIndexClient::getType(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getType";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getSame(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getSame";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

std::vector<int64_t> IndividualIndexClient::getDistincts(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getDistincts";
  srv.request.param = std::to_string(index);

  return callIndexes(srv);
}

} // namespace onto