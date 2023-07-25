#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/OntologyIndexClient.h"

namespace onto {

std::vector<int64_t> OntologyIndexClient::getUp(int64_t index, int depth, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getUp";
  srv.request.param = std::to_string(index);
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  if(depth >= 0)
    srv.request.param += " -d " + std::to_string(depth);

  return callIndexes(srv);
}

bool OntologyIndexClient::isA(int64_t index, int64_t base_class)
{
  std::vector<int64_t> res = getUp(index, -1, base_class);
  if(res.size() == 0)
    return false;
  else
    return true;
}

std::string OntologyIndexClient::getName(int64_t index, bool take_id)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getName";
  srv.request.param = std::to_string(index);
  if(take_id == false)
    srv.request.param += " -i false";

  return callStr(srv);
}

std::vector<std::string> OntologyIndexClient::getNames(int64_t index, bool take_id)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getNames";
  srv.request.param = std::to_string(index);
  if(take_id == false)
    srv.request.param += " -i false";

  return call(srv);
}

std::vector<std::string> OntologyIndexClient::getEveryNames(int64_t index, bool take_id)
{
  ontologenius::OntologeniusIndexService srv;
  srv.request.action = "getEveryNames";
  srv.request.param = std::to_string(index);
  if(take_id == false)
    srv.request.param += " -i false";

  return call(srv);
}

std::vector<int64_t> OntologyIndexClient::find(const std::string& name, bool take_id, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;

  srv.request.action = "find";
  srv.request.param = name;
  if(take_id == false)
    srv.request.param += " -i false";
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return callIndexes(srv);
}

std::vector<int64_t> OntologyIndexClient::findSub(const std::string& name, bool take_id, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;

  srv.request.action = "findSub";
  srv.request.param = name;
  if(take_id == false)
    srv.request.param += " -i false";
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return callIndexes(srv);
}

std::vector<int64_t> OntologyIndexClient::findRegex(const std::string& regex, bool take_id, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;

  srv.request.action = "findRegex";
  srv.request.param = regex;
  if(take_id == false)
    srv.request.param += " -i false";
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return callIndexes(srv);
}

std::vector<std::string> OntologyIndexClient::findFuzzy(const std::string& name, double threshold, bool take_id, int64_t selector)
{
  ontologenius::OntologeniusIndexService srv;

  srv.request.action = "findFuzzy";
  srv.request.param = name + " -t " + std::to_string(threshold);
  if(take_id == false)
    srv.request.param += " -i false";
  if(selector != 0)
    srv.request.param += " -s " + std::to_string(selector);

  return call(srv);
}

bool OntologyIndexClient::exist(int64_t index)
{
  ontologenius::OntologeniusIndexService srv;

  srv.request.action = "exist";
  srv.request.param = std::to_string(index);

  return (callIndex(srv) != 0);
}

} // namespace onto
