#include "ontologenius/API/ontologenius/clients/ManagerClient.h"

#include <vector>
#include <string>

namespace onto {

  std::vector<std::string> ManagerClient::list()
  {
    return call("list", "");
  }

  bool ManagerClient::add(const std::string& name)
  {
    return callNR("add", name);
  }

  bool ManagerClient::copy(const std::string& dest_name, const std::string& src_name)
  {
    return callBool("copy", dest_name + "=" + src_name);
  }

  bool ManagerClient::del(const std::string& name)
  {
    return callNR("delete", name);
  }

  std::vector<std::string> ManagerClient::getDifference(const std::string& onto1, const std::string& onto2, const std::string& concept)
  {
    std::string param = onto1 + "|" + onto2 + "|" + concept;
    return call("difference", param);
  }

} // namespace onto