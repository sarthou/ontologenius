#include "ontologenius/API/ontologenius/clients/ontologyClients/OntologyClient.h"

namespace onto {

  std::vector<std::string> OntologyClient::getUp(const std::string& name, int depth, const std::string& selector)
  {
    std::string param = name;
    if(selector != "")
      param += " -s " + selector;

    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getUp", param);
  }

  bool OntologyClient::isA(const std::string& name, const std::string& base_class)
  {
    std::vector<std::string> res = getUp(name, -1, base_class);
    if(res.size() == 0)
      return false;
    else
      return true;
  }

  std::string OntologyClient::getName(const std::string& name, bool take_id)
  {
    std::string param = name;
    if(take_id == false)
      param += " -i false";

    return callStr("getName", param);
  }

  std::vector<std::string> OntologyClient::getNames(const std::string& name, bool take_id)
  {
    std::string param = name;
    if(take_id == false)
      param += " -i false";

    return call("getNames", param);
  }

  std::vector<std::string> OntologyClient::getEveryNames(const std::string& name, bool take_id)
  {
    std::string param = name;
    if(take_id == false)
      param += " -i false";

    return call("getEveryNames", param);
  }

  std::vector<std::string> OntologyClient::find(const std::string& name, bool take_id, const std::string& selector)
  {
    std::string param = name;
    if(take_id == false)
      param += " -i false";
    if(selector != "")
      param += " -s " + selector;

    return call("find", param);
  }

  std::vector<std::string> OntologyClient::findSub(const std::string& name, bool take_id, const std::string& selector)
  {
    std::string param = name;
    if(take_id == false)
      param += " -i false";
    if(selector != "")
      param += " -s " + selector;

    return call("findSub", param);
  }

  std::vector<std::string> OntologyClient::findRegex(const std::string& regex, bool take_id, const std::string& selector)
  {
    std::string param = regex;
    if(take_id == false)
      param += " -i false";
    if(selector != "")
      param += " -s " + selector;

    return call("findRegex", param);
  }

  std::vector<std::string> OntologyClient::findFuzzy(const std::string& name, double threshold, bool take_id, const std::string& selector)
  {
    std::string param = name + " -t " + std::to_string(threshold);
    if(take_id == false)
      param += " -i false";
    if(selector != "")
      param += " -s " + selector;

    return call("findFuzzy", param);
  }

  bool OntologyClient::exist(const std::string& name)
  {
    std::string param = name;
    return (callStr("exist", param) != "");
  }

} // namespace onto