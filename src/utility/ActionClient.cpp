#include "ontoloGenius/utility/ActionClient.h"

bool ActionClient::close()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "close";

  return callNR(srv);
}

bool ActionClient::setLang(const std::string& lang)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "setLang";
  srv.request.param = lang;

  return callNR(srv);
}

bool ActionClient::add(const std::string& uri)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "add";
  srv.request.param = uri;

  return callNR(srv);
}

bool ActionClient::fadd(const std::string& file)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "close";
  srv.request.param = file;

  return callNR(srv);
}

bool ActionClient::reset()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "reset";

  return callNR(srv);
}
