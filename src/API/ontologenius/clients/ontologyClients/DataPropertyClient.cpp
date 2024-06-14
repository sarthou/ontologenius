#include "ontologenius/API/ontologenius/clients/ontologyClients/DataPropertyClient.h"

#include <vector>
#include <string>

namespace onto {

  std::vector<std::string> DataPropertyClient::getDown(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getDown", param);
  }

  std::vector<std::string> DataPropertyClient::getDisjoint(const std::string& name)
  {
    return call("getDisjoint", name);
  }

  std::vector<std::string> DataPropertyClient::getDomain(const std::string& name, int depth)
  {
    std::string param = name;
    if(depth >= 0)
      param += " -d " + std::to_string(depth);

    return call("getDomain", param);
  }

  std::vector<std::string> DataPropertyClient::getRange(const std::string& name)
  {
    return call("getRange", name);
  }

} // namespace onto