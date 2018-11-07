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

  return callNR(srv);
}

bool ArguerClient::deactivate(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "deactivate";

  return callNR(srv);
}

std::string ArguerClient::getDescription(const std::string& file)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDescription";

  return callStr(srv);
}
