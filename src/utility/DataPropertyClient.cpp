#include "ontoloGenius/utility/DataPropertyClient.h"

std::vector<std::string> DataPropertyClient::getDown(const std::string& name, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDisjoint";
  srv.request.param = name;

  if(depth >= 0)
    srv.request.param += " < " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> DataPropertyClient::getDisjoint(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDisjoint";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> DataPropertyClient::getDomain(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDomain";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> DataPropertyClient::getRange(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRange";
  srv.request.param = name;

  return call(srv);
}
