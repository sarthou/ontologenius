#ifndef CLIENTBASE_H
#define CLIENTBASE_H

#include "ontologenius/OntologeniusService.h"

#include "ros/ros.h"

#include <vector>
#include <string>

class ClientBase
{
public:
  ClientBase(ros::NodeHandle* n, std::string name) : client(n->serviceClient<ontologenius::OntologeniusService>("ontologenius/" + name, true))
  {
    n_ = n;
  }

  size_t nb() {return cpt;}
  void reset() {cpt = 0;}

  std::vector<std::string> getUp(std::string& name, int depth = -1, const std::string& selector = "");
  bool isA(std::string& name, const std::string& base_class);
  std::string getName(const std::string& name);
  std::vector<std::string> find(const std::string& name);

protected:
  ros::ServiceClient client;

  inline std::vector<std::string> call(ontologenius::OntologeniusService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(client.call(srv))
      return srv.response.values;
    else
      return res;
  }

private:
    ros::NodeHandle* n_;
    static size_t cpt;
};

#endif
