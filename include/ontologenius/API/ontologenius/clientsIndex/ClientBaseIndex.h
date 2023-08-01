#ifndef ONTOLOGENIUS_CLIENTBASEINDEX_H
#define ONTOLOGENIUS_CLIENTBASEINDEX_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include "ontologenius/OntologeniusIndexService.h"

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

namespace onto {

/// @brief The ClientBaseIndex class provides an abstraction for any ROS services based on indexes.
/// This class ensure a persistent connection with the service based on.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class ClientBaseIndex
{
public:
  /// @brief Constructs a ROS client.
  /// @param n is an initialized ROS node handle.
  /// @param name is the name of the ontologenius service
  ClientBaseIndex(ros::NodeHandle* n, const std::string& name) : client(n->serviceClient<ontologenius::OntologeniusIndexService>("/ontologenius/" + name, true)), name_(name)
  {
    n_ = n;
  }

  /// @brief Gives the total number of service calls from all ClientBaseIndex instances since the last reset.
  size_t nb() {return cpt;}
  /// @brief Reset Call Counter for all instances of ClientBaseIndex.
  void resetNb() {cpt = 0;}
  static void verbose(bool verbose) { verbose_ = verbose; }

protected:
  ros::ServiceClient client;

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param srv is the request.
  /// @return Returns a list of string. If the service call fails, the first element of the returned vector is "ERR:SERVICE_FAIL".
  inline std::vector<std::string> call(ontologenius::OntologeniusIndexService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(client.call(srv))
      return srv.response.string_values;
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusIndexService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        return srv.response.string_values;
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

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param srv is the request.
  /// @return Returns a list of int64. If the service call fails, the first element of the returned vector is "0".
  inline std::vector<int64_t> callIndexes(ontologenius::OntologeniusIndexService& srv)
  {
    std::vector<int64_t> res;
    cpt++;

    if(client.call(srv))
      return srv.response.index_values;
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusIndexService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        return srv.response.index_values;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        res.push_back(0);
        return res;
      }
    }
  }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param srv is the request.
  /// @return Returns a single string. If the service call fails, the returned value is "ERR:SERVICE_FAIL".
  inline std::string callStr(ontologenius::OntologeniusIndexService& srv)
  {
    cpt++;

    if(client.call(srv))
    {
      if(srv.response.string_values.size())
        return srv.response.string_values[0];
      else
        return "";
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusIndexService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        if(srv.response.string_values.size())
          return srv.response.string_values[0];
        else
          return "";
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        return "ERR:SERVICE_FAIL";
      }
    }
  }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param srv is the request.
  /// @return Returns a single int64. If the service call fails or has no response, the returned value is "0".
  inline int64_t callIndex(ontologenius::OntologeniusIndexService& srv)
  {
    cpt++;

    if(client.call(srv))
    {
      if(srv.response.index_values.size())
        return srv.response.index_values[0];
      else
        return 0;
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusIndexService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        if(srv.response.index_values.size())
          return srv.response.index_values[0];
        else
          return 0;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        return 0;
      }
    }
  }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param srv is the request.
  /// @return Returns false if the service call fails.
  inline bool callNR(ontologenius::OntologeniusIndexService& srv)
  {
    cpt++;

    if(client.call(srv))
      return true;
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusIndexService>("/ontologenius/" + name_, true);
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

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param srv is the request.
  /// @return Returns false if the service call fails or the result code of the service is different from SUCCESS.
  inline bool callBool(ontologenius::OntologeniusIndexService& srv)
  {
    cpt++;

    if(client.call(srv))
      return (srv.response.code == 0);
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<ontologenius::OntologeniusIndexService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        return (srv.response.code == 0);
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

} // namespace onto

#endif // ONTOLOGENIUS_CLIENTBASEINDEX_H
