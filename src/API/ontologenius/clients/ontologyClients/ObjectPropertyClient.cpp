#include "ontologenius/API/ontologenius/clients/ontologyClients/ObjectPropertyClient.h"

#include <string>
#include <vector>

namespace onto {

  std::vector<std::string> ObjectPropertyClient::getDown(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getDown", param);
  }

  std::vector<std::string> ObjectPropertyClient::getDisjoint(const std::string& name)
  {
    return call("getDisjoint", name);
  }

  std::vector<std::string> ObjectPropertyClient::getDomain(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getDomain", param);
  }

  std::vector<std::string> ObjectPropertyClient::getRange(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getRange", param);
  }

  std::vector<std::string> ObjectPropertyClient::getInverse(const std::string& name)
  {
    return call("getInverse", name);
  }

} // namespace onto