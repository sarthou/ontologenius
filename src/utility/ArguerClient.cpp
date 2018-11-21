#include "ontoloGenius/utility/ArguerClient.h"

std::vector<std::string> ArguerClient::list()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "list";

  return call(srv);
}

bool ArguerClient::activate(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "activate";
  srv.request.param = name;

  return callNR(srv);
}

bool ArguerClient::deactivate(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "deactivate";
  srv.request.param = name;

  return callNR(srv);
}

std::string ArguerClient::getDescription(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDescription";
  srv.request.param = name;

  return callStr(srv);
}
