#include "ontologenius/API/ontologenius/clients/ontologyClients/ObjectPropertyClient.h"

std::vector<std::string> ObjectPropertyClient::getDown(const std::string& name, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDown";
  srv.request.param = name;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> ObjectPropertyClient::getDisjoint(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDisjoint";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> ObjectPropertyClient::getDomain(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDomain";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> ObjectPropertyClient::getRange(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRange";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> ObjectPropertyClient::getInverse(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getInverse";
  srv.request.param = name;

  return call(srv);
}
