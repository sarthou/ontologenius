#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/OntologyIndexClient.h"

namespace onto {

std::vector<int64_t> OntologyIndexClient::getUp(int64_t index, int depth, int64_t selector)
{
  std::string param = std::to_string(index);
  if(selector != 0)
    param += " -s " + std::to_string(selector);

  if(depth >= 0)
    param += " -d " + std::to_string(depth);

  return callIndexes("getUp", param);
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
  std::string param = std::to_string(index);
  if(take_id == false)
    param += " -i false";

  return callStr("getName", param);
}

std::vector<std::string> OntologyIndexClient::getNames(int64_t index, bool take_id)
{
  std::string param = std::to_string(index);
  if(take_id == false)
    param += " -i false";

  return call("getNames", param);
}

std::vector<std::string> OntologyIndexClient::getEveryNames(int64_t index, bool take_id)
{
  std::string param = std::to_string(index);
  if(take_id == false)
    param += " -i false";

  return call("getEveryNames", param);
}

std::vector<int64_t> OntologyIndexClient::find(const std::string& name, bool take_id, int64_t selector)
{
  std::string param = name;
  if(take_id == false)
    param += " -i false";
  if(selector != 0)
    param += " -s " + std::to_string(selector);

  return callIndexes("find", param);
}

std::vector<int64_t> OntologyIndexClient::findSub(const std::string& name, bool take_id, int64_t selector)
{
  std::string param = name;
  if(take_id == false)
    param += " -i false";
  if(selector != 0)
    param += " -s " + std::to_string(selector);

  return callIndexes("findSub", param);
}

std::vector<int64_t> OntologyIndexClient::findRegex(const std::string& regex, bool take_id, int64_t selector)
{
  std::string param = regex;
  if(take_id == false)
    param += " -i false";
  if(selector != 0)
    param += " -s " + std::to_string(selector);

  return callIndexes("findRegex", param);
}

std::vector<std::string> OntologyIndexClient::findFuzzy(const std::string& name, double threshold, bool take_id, int64_t selector)
{
  std::string param = name + " -t " + std::to_string(threshold);
  if(take_id == false)
    param += " -i false";
  if(selector != 0)
    param += " -s " + std::to_string(selector);

  return call("findFuzzy", param);
}

bool OntologyIndexClient::exist(int64_t index)
{
  std::string param = std::to_string(index);
  return (callIndex("exist", param) != 0);
}

} // namespace onto
