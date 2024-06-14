#include "ontologenius/API/ontologenius/clients/ReasonerClient.h"

#include <string>
#include <vector>

namespace onto {

  std::vector<std::string> ReasonerClient::list()
  {
    return call("list", "");
  }

  std::vector<std::string> ReasonerClient::activeList()
  {
    return call("activeList", "");
  }

  bool ReasonerClient::activate(const std::string& name)
  {
    return callNR("activate", name);
  }

  bool ReasonerClient::deactivate(const std::string& name)
  {
    return callNR("deactivate", name);
  }

  std::string ReasonerClient::getDescription(const std::string& name)
  {
    return callStr("getDescription", name);
  }

} // namespace onto