#include "ontologenius/API/ontologenius/clients/ActionClient.h"

namespace onto {

bool ActionClient::close()
{
  if(callNR("close", "") == false)
    return false;
  else
    return (getErrorCode() == 0);
}

bool ActionClient::save(const std::string& path)
{
  return callNR("save", path);
}

bool ActionClient::exportToXml(const std::string& path)
{
  return callNR("export", path);
}

bool ActionClient::setLang(const std::string& lang)
{
  return callNR("setLang", lang);
}

std::string ActionClient::getLang()
{
  return callStr("getLang", "");
}

bool ActionClient::add(const std::string& uri)
{
  return callNR("add", uri);
}

bool ActionClient::fadd(const std::string& file)
{
  return callNR("fadd", file);
}

bool ActionClient::reset()
{
  return callNR("reset", "");
}

bool ActionClient::clear()
{
  return callNR("clear", "");
}

} // namespace onto