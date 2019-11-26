#include "ontoloGenius/utility/clients/ManagerClient.h"

std::vector<std::string> ManagerClient::list()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "list";

  return call(srv);
}

bool ManagerClient::add(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "add";
  srv.request.param = name;

  return callNR(srv);
}

bool ManagerClient::copy(const std::string& dest_name, const std::string& src_name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "copy";
  srv.request.param = dest_name + "=" + src_name;

  return callBool(srv);
}

bool ManagerClient::del(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "delete";
  srv.request.param = name;

  return callNR(srv);
}

std::vector<std::string> ManagerClient::getDifference(const std::string& onto1, const std::string& onto2, const std::string& concept)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "difference";
  srv.request.param = onto1 + "|" + onto2 + "|" + concept;

  return call(srv);
}
