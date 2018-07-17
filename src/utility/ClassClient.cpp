#include "ontoloGenius/utility/ClassClient.h"

std::vector<std::string> ClassClient::getDown(const std::string& name, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDisjoint";
  srv.request.param = name;

  if(depth >= 0)
    srv.request.param += " < " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> ClassClient::getDisjoint(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDisjoint";
  srv.request.param = name;

  return call(srv);
}
