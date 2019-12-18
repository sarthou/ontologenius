#include "ontologenius/API/ontologenius/clients/ontologyClients/ClassClient.h"

std::vector<std::string> ClassClient::getDown(const std::string& name, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDown";
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

std::vector<std::string> ClassClient::getOn(const std::string& name, const std::string& property, const std::string& selector)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getOn";
  srv.request.param = name + ":" + property;
  if(selector != "")
    srv.request.param += " -s " + selector;

  return call(srv);
}

std::vector<std::string> ClassClient::getFrom(const std::string& property, const std::string& name, const std::string& selector)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getFrom";
  srv.request.param = name + ":" + property;
  if(selector != "")
    srv.request.param += " -s " + selector;

  return call(srv);
}

std::vector<std::string> ClassClient::getWith(const std::string& indiv_1, const std::string& indiv_2, const std::string& selector, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getWith";
  srv.request.param = indiv_1 + ":" + indiv_2;
  if(selector != "")
    srv.request.param += " -s " + selector;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> ClassClient::getRelatedFrom(const std::string& property)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelatedFrom";
  srv.request.param = property;

  return call(srv);
}

std::vector<std::string> ClassClient::getRelatedOn(const std::string& property)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelatedOn";
  srv.request.param = property;

  return call(srv);
}

std::vector<std::string> ClassClient::getRelatedWith(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelatedWith";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> ClassClient::getRelationFrom(const std::string& name, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelationFrom";
  srv.request.param = name;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> ClassClient::getRelationOn(const std::string& name, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelationOn";
  srv.request.param = name;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> ClassClient::getRelationWith(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelationWith";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> ClassClient::getDomainOf(const std::string& name, const std::string& selector, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDomainOf";
  srv.request.param = name;

  if(selector != "")
    srv.request.param += " -s " + selector;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}

std::vector<std::string> ClassClient::getRangeOf(const std::string& name, const std::string& selector, int depth)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRangeOf";
  srv.request.param = name;
  
  if(selector != "")
    srv.request.param += " -s " + selector;

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return call(srv);
}
