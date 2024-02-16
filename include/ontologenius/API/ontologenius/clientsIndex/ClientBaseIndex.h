#ifndef ONTOLOGENIUS_CLIENTBASEINDEX_H
#define ONTOLOGENIUS_CLIENTBASEINDEX_H

#include <vector>
#include <string>
#include <iostream>

#include "ontologenius/compat/ros.h"

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
  static int16_t ignore_;
public:
  /// @brief Constructs a ROS client.
  /// @param name is the name of the ontologenius service
  explicit ClientBaseIndex(const std::string& name) : name_(name),
                                                      error_code_(0),
                                                      client_("/ontologenius/" + name)
  {}

  /// @brief Gives the total number of service calls from all ClientBaseIndex instances since the last reset.
  size_t nb() {return cpt;}
  /// @brief Reset Call Counter for all instances of ClientBaseIndex.
  void resetNb() {cpt = 0;}
  static void verbose(bool verbose) { verbose_ = verbose; }

  int getErrorCode() { return error_code_; }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns a list of string. If the service call fails, the first element of the returned vector is "ERR:SERVICE_FAIL".
  inline std::vector<std::string> call(const std::string& action, const std::string& param, int16_t &code = ignore_)
  {
    cpt++;

    auto req = ontologenius::compat::make_request<ontologenius::compat::OntologeniusIndexService>();
    auto res = ontologenius::compat::make_response<ontologenius::compat::OntologeniusIndexService>();

    [action, param](auto&& req)
    {
      req->action = action;
      req->param = param;
    }(ontologenius::compat::onto_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_)::Status;

    switch (client_.call(req, res)) {
      case ResultTy::SUCCESSFUL_WITH_RETRIES:
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        [[fallthrough]];
      }
      case ResultTy::SUCCESSFUL:
      {
        return [&](auto&& res)
        {
          code = res->code;
          error_code_ = res->code;
          return res->string_values;
        }(ontologenius::compat::onto_ros::getServicePointer(res));
      }
      case ResultTy::FAILURE:
      {
        if(verbose_)
          std::cout << COLOR_ORANGE << "Failed to call call ontologenius/" << name_ << COLOR_OFF << std::endl;
        [[fallthrough]];
      }
      default:
      {
        return { "ERR:SERVICE_FAIL" };
      }
    }
  }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns a list of int64. If the service call fails, the first element of the returned vector is "0".
  inline std::vector<int64_t> callIndexes(const std::string& action, const std::string& param, int16_t &code = ignore_)
  {
    cpt++;

    auto req = ontologenius::compat::make_request<ontologenius::compat::OntologeniusIndexService>();
    auto res = ontologenius::compat::make_response<ontologenius::compat::OntologeniusIndexService>();

    [action, param](auto&& req)
    {
      req->action = action;
      req->param = param;
    }(ontologenius::compat::onto_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_)::Status;

    switch (client_.call(req, res))
    {
      case ResultTy::SUCCESSFUL_WITH_RETRIES:
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        [[fallthrough]];
      }
      case ResultTy::SUCCESSFUL:
      {
        return [&](auto&& res)
        {
          code = res->code;
          error_code_ = res->code;
          return res->index_values;
        }(ontologenius::compat::onto_ros::getServicePointer(res));
      }
      case ResultTy::FAILURE:
      {
        if(verbose_)
          std::cout << COLOR_ORANGE << "Failed to call call ontologenius/" << name_ << COLOR_OFF << std::endl;
        [[fallthrough]];
      }
      default:
      {
        return { 0 };
      }
    }
  }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns a single string. If the service call fails, the returned value is "ERR:SERVICE_FAIL".
  inline std::string callStr(const std::string& action, const std::string& param, int16_t& code = ignore_)
  {
    auto res = this->call(action, param, code);
    return res.empty() ? "" : res[0];
  }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns a single int64. If the service call fails or has no response, the returned value is "0".
  inline int64_t callIndex(const std::string& action, const std::string& param)
  {
    auto res = this->callIndexes(action, param);
    return res.empty() ? 0 : res[0];
  }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns false if the service call fails.
  inline bool callNR(const std::string& action, const std::string& param)
  {
    return this->callStr(action, param) != "ERR:SERVICE_FAIL";
  }

  /// @brief Calls the service set up in the constructor of ClientBaseIndex.
  /// @param action the query action.
  /// @param param the query parameters.
  /// @return Returns false if the service call fails or the result code of the service is different from SUCCESS.
  inline bool callBool(const std::string& action, const std::string& param)
  {
    int16_t code;
    auto res = this->callStr(action, param, code);
    return (res != "ERR:SERVICE_FAIL") && (code == 0);
  }

private:
  std::string name_;
  static size_t cpt;
  static bool verbose_;
  int error_code_;
protected:
  ontologenius::compat::onto_ros::Client<ontologenius::compat::OntologeniusIndexService> client_;
};

} // namespace onto

#endif // ONTOLOGENIUS_CLIENTBASEINDEX_H
