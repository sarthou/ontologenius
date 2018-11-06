#include "ontoloGenius/utility/ClientBase.h"

size_t ClientBase::cpt = 0;


std::vector<std::string> ClientBase::getUp(std::string& name, int depth, const std::string& selector)
{
  ontologenius::OntologeniusService srv;
  if(selector == "")
  {
    srv.request.action = "getUp";
    srv.request.param = name;
  }
  else
  {
    srv.request.action = "select:getUp";
    srv.request.param = selector + "=" + name;
  }

  if(depth >= 0)
    srv.request.param += " < " + std::to_string(depth);

  return call(srv);
}

bool ClientBase::isA(std::string& name, const std::string& base_class)
{
  std::vector<std::string> res = getUp(name, -1, base_class);
  if(res.size() == 0)
    return false;
  else
    return true;
}

std::string ClientBase::getName(const std::string& name)
{
  std::string res = "";

  ontologenius::OntologeniusService srv;
  srv.request.action = "getName";
  srv.request.param = name;

  cpt++;

  if(client.call(srv))
  {
    if(srv.response.values.size())
      return srv.response.values[0];
    else
      return res;
  }
  else
  {
    std::cout << COLOR_ORANGE << "Failure to call ontoloGenius services" << COLOR_OFF << std::endl;
    client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/" + name_, true);
    if(client.call(srv))
    {
      std::cout << COLOR_GREEN << "Restored ontoloGenius services" << COLOR_OFF << std::endl;
      if(srv.response.values.size())
        return srv.response.values[0];
      else
        return res;
    }
    else
    {
      std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
      res = "ERR:SERVICE_FAIL";
      return res;
    }
  }
}

std::vector<std::string> ClientBase::find(const std::string& name)
{
  ontologenius::OntologeniusService srv;

  srv.request.action = "find";
  srv.request.param = name;

  return call(srv);
}
