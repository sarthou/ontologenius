#include "ontologenius/API/ontologenius/clients/ReasonerClient.h"

std::vector<std::string> ReasonerClient::list()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "list";

  return call(srv);
}

std::vector<std::string> ReasonerClient::activeList()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "activeList";

  return call(srv);
}

bool ReasonerClient::activate(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "activate";
  srv.request.param = name;

  return callNR(srv);
}

bool ReasonerClient::deactivate(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "deactivate";
  srv.request.param = name;

  return callNR(srv);
}

std::string ReasonerClient::getDescription(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDescription";
  srv.request.param = name;

  return callStr(srv);
}
