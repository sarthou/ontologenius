#ifndef ONTOLOGENIUS_CLIENTBASE_H
#define ONTOLOGENIUS_CLIENTBASE_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include "ontologenius/OntologeniusService.h"

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE  "\x1B[1;33m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN   "\x1B[1;92m"
#endif

class ClientBase
{
public:
  ClientBase(ros::NodeHandle* n, std::string name) : client(n->serviceClient<ontologenius::OntologeniusService>("ontologenius/" + name, true))
  {
    n_ = n;
    name_ = name;
  }

  size_t nb() {return cpt;}
  void resetNb() {cpt = 0;}
  static void verbose(bool verbose) { verbose_ = verbose; }

protected:
  ros::ServiceClient client;

  inline std::vector<std::string> call(ontologenius::OntologeniusService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(client.call(srv))
      return srv.response.values;
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        return srv.response.values;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        res.push_back("ERR:SERVICE_FAIL");
        return res;
      }
    }
  }

  inline std::string callStr(ontologenius::OntologeniusService& srv)
  {
    std::string res = "";
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
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        if(srv.response.values.size())
          return srv.response.values[0];
        else
          return res;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        res = "ERR:SERVICE_FAIL";
        return res;
      }
    }
  }

  inline bool callNR(ontologenius::OntologeniusService& srv)
  {
    cpt++;

    if(client.call(srv))
      return true;
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        return true;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        return false;
      }
    }
  }

private:
    std::string name_;
    ros::NodeHandle* n_;
    static size_t cpt;
    static bool verbose_;
};

#endif // ONTOLOGENIUS_CLIENTBASE_H
