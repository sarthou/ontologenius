#include "ontologenius/API/ontologenius/clients/ontologyClients/IndividualClient.h"

std::vector<std::string> IndividualClient::getOn(const std::string& name, const std::string& property, const std::string& selector)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getOn";
  srv.request.param = name + ":" + property;
  if(selector != "")
    srv.request.param += " -s " + selector;

  return call(srv);
}

std::vector<std::string> IndividualClient::getFrom(const std::string& property, const std::string& name, const std::string& selector)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getFrom";
  srv.request.param = name + ":" + property;
  if(selector != "")
    srv.request.param += " -s " + selector;

  return call(srv);
}

std::vector<std::string> IndividualClient::getWith(const std::string& indiv_from, const std::string& indiv_to, const std::string& selector, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getWith";
  srv.request.param = indiv_from + ":" + indiv_to;
  if(selector != "")
    srv.request.param += " -s " + selector;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> IndividualClient::getRelatedFrom(const std::string& property)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelatedFrom";
  srv.request.param = property;

  return call(srv);
}

std::vector<std::string> IndividualClient::getRelatedOn(const std::string& property)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelatedOn";
  srv.request.param = property;

  return call(srv);
}

std::vector<std::string> IndividualClient::getRelatedWith(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelatedWith";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> IndividualClient::getRelationFrom(const std::string& name, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelationFrom";
  srv.request.param = name;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> IndividualClient::getRelationOn(const std::string& name, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelationOn";
  srv.request.param = name;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> IndividualClient::getRelationWith(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelationWith";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> IndividualClient::getType(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getType";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> IndividualClient::getSame(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getSame";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> IndividualClient::getDistincts(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDistincts";
  srv.request.param = name;

  return call(srv);
}
