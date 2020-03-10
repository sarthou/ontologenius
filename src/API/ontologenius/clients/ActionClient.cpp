#include "ontologenius/API/ontologenius/clients/ActionClient.h"

bool ActionClient::close()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "close";

  return callNR(srv);
}

bool ActionClient::save(const std::string& path)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "save";
  srv.request.param = path;

  return callNR(srv);
}

bool ActionClient::exportToXml(const std::string& path)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "export";
  srv.request.param = path;

  return callNR(srv);
}

bool ActionClient::setLang(const std::string& lang)
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "setLang";
  srv.request.param = lang;

  return callNR(srv);
}

std::string ActionClient::getLang()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "getLang";

  return callStr(srv);
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
  srv.request.action = "fadd";
  srv.request.param = file;

  return callNR(srv);
}

bool ActionClient::reset()
{
  ontologenius::OntologeniusService srv;
  srv.request.action = "reset";

  return callNR(srv);
}
