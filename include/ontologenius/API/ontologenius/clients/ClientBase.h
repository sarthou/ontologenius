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

namespace onto {

/// @brief The ClientBase class provides an abstraction for any ROS services.
/// This class ensure a persistent connection with the service based on.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class ClientBase
{
public:
  /// @brief Constructs a ROS client.
  /// @param name is the name of the ontologenius service
  explicit ClientBase(const std::string& name) : name_(name),
                                                 error_code_(0),
                                                 client(n_.serviceClient<ontologenius::OntologeniusService>("/ontologenius/" + name, true))
                                                 
  {}

  /// @brief Gives the total number of service calls from all ClientBase instances since the last reset.
  size_t nb() {return cpt;}
  /// @brief Reset Call Counter for all instances of ClientBase.
  void resetNb() {cpt = 0;}
  static void verbose(bool verbose) { verbose_ = verbose; }

  int getErrorCode() { return error_code_; }

  /// @brief Calls the service set up in the constructor of ClientBase.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns a list of string. If the service call fails, the first element of the returned vector is "ERR:SERVICE_FAIL".
  inline std::vector<std::string> call(const std::string& action, const std::string& param)
  {
    ontologenius::OntologeniusService srv;
    srv.request.action = action;
    srv.request.param = param;
    std::vector<std::string> res;
    cpt++;

    if(client.call(srv))
    {
      error_code_ = srv.response.code;
      return srv.response.values;
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_.serviceClient<ontologenius::OntologeniusService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        error_code_ = srv.response.code;
        return srv.response.values;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        error_code_ = -1;
        res.push_back("ERR:SERVICE_FAIL");
        return res;
      }
    }
  }

  /// @brief Calls the service set up in the constructor of ClientBase.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns a single string. If the service call fails, the returned value is "ERR:SERVICE_FAIL".
  inline std::string callStr(const std::string& action, const std::string& param)
  {
    ontologenius::OntologeniusService srv;
    srv.request.action = action;
    srv.request.param = param;
    std::string res = "";
    cpt++;

    if(client.call(srv))
    {
      error_code_ = srv.response.code;
      if(srv.response.values.size())
        return srv.response.values[0];
      else
        return res;
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_.serviceClient<ontologenius::OntologeniusService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        error_code_ = srv.response.code;
        if(srv.response.values.size())
          return srv.response.values[0];
        else
          return res;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        error_code_ = -1;
        res = "ERR:SERVICE_FAIL";
        return res;
      }
    }
  }

  /// @brief Calls the service set up in the constructor of ClientBase.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns false if the service call fails.
  inline bool callNR(const std::string& action, const std::string& param)
  {
    ontologenius::OntologeniusService srv;
    srv.request.action = action;
    srv.request.param = param;
    cpt++;

    if(client.call(srv))
    {
      error_code_ = srv.response.code;
      return true;
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_.serviceClient<ontologenius::OntologeniusService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        error_code_ = srv.response.code;
        return true;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        error_code_ = -1;
        return false;
      }
    }
  }

  /// @brief Calls the service set up in the constructor of ClientBase.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns false if the service call fails or the result code of the service is different from SUCCESS.
  inline bool callBool(const std::string& action, const std::string& param)
  {
    ontologenius::OntologeniusService srv;
    srv.request.action = action;
    srv.request.param = param;
    cpt++;

    if(client.call(srv))
    {
      error_code_ = srv.response.code;
      return (srv.response.code == 0);
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      client = n_.serviceClient<ontologenius::OntologeniusService>("/ontologenius/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        error_code_ = srv.response.code;
        return (srv.response.code == 0);
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        error_code_ = -1;
        return false;
      }
    }
  }

private:
    std::string name_;
    ros::NodeHandle n_;
    static size_t cpt;
    static bool verbose_;
    int error_code_;

protected:
  ros::ServiceClient client;
};

} // namespace onto

#endif // ONTOLOGENIUS_CLIENTBASE_H
