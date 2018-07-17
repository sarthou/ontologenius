#include "ontoloGenius/utility/IndividualClient.h"

std::vector<std::string> IndividualClient::getOn(const std::string& name, const std::string& property)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getOn";
  srv.request.param = name + ":" + property;

  return call(srv);
}

std::vector<std::string> IndividualClient::getFrom(const std::string& property, const std::string& name, const std::string& selector)
{
  ontologenius::OntologeniusService srv;
  if(selector == "")
  {
    srv.request.action = "getFrom";
    srv.request.param = name + ":" + property;
  }
  else
  {
    srv.request.action = "select:getFrom";
    srv.request.param = selector + "=" + name + ":" + property;
  }

  return call(srv);
}

std::vector<std::string> IndividualClient::getWith(const std::string& indiv_1, const std::string& indiv_2, const std::string& selector)
{
  ontologenius::OntologeniusService srv;
  if(selector == "")
  {
    srv.request.action = "getWith";
    srv.request.param = indiv_1 + ":" + indiv_2;
  }
  else
  {
    srv.request.action = "select:getWith";
    srv.request.param = selector + "=" + indiv_1 + ":" + indiv_2;
  }

  return call(srv);
}

std::vector<std::string> IndividualClient::getDown(std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getDown";
  srv.request.param = name;

  return call(srv);
}

std::vector<std::string> IndividualClient::getRelatedFrom(const std::string& name)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelatedFrom";
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
