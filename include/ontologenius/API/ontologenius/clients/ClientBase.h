#ifndef ONTOLOGENIUS_CLIENTBASE_H
#define ONTOLOGENIUS_CLIENTBASE_H

#include <string>
#include <vector>

#include "ontologenius/compat/ros.h"

#ifndef COLOR_OFF
#define COLOR_OFF "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED "\x1B[0;91m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE "\x1B[1;33m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN "\x1B[1;92m"
#endif

namespace onto {

  /// @brief The ClientBase class provides an abstraction for any ROS services.
  /// This class ensure a persistent connection with the service based on.
  /// The persistent connection ensures a minimal response time.
  /// A reconnection logic is implemented in the event that the persistent connection fails.
  class ClientBase
  {
    static int16_t ignore;

  public:
    /// @brief Constructs a ROS client.
    /// @param name is the name of the ontologenius service
    explicit ClientBase(const std::string& name) : name_(name),
                                                   error_code_(0),
                                                   client_("/ontologenius/" + name)

    {}

    /// @brief Gives the total number of service calls from all ClientBase instances since the last reset.
    size_t nb() { return cpt; }
    /// @brief Reset Call Counter for all instances of ClientBase.
    void resetNb() { cpt = 0; }
    static void verbose(bool verbose) { client_verbose = verbose; }

    int getErrorCode() const { return error_code_; }

    /// @brief Calls the service set up in the constructor of ClientBase.
    /// @param action the query action.
    /// @param param the query parameters.
    /// @return Returns a list of string. If the service call fails, the first element of the returned vector is "ERR:SERVICE_FAIL".
    std::vector<std::string> call(const std::string& action, const std::string& param, int16_t& code = ignore)
    {
      cpt++;

      auto req = ontologenius::compat::makeRequest<ontologenius::compat::OntologeniusService>();
      auto res = ontologenius::compat::makeResponse<ontologenius::compat::OntologeniusService>();

      [action, param](auto&& req) {
        req->action = action;
        req->param = param;
      }(ontologenius::compat::onto_ros::getServicePointer(req));

      using ResultTy = typename decltype(client_)::Status_e;

      switch(client_.call(req, res))
      {
      case ResultTy::ros_status_successful_with_retry:
      {
        if(client_verbose)
          std::cout << COLOR_GREEN << "Restored ontologenius/" << name_ << COLOR_OFF << std::endl;
        [[fallthrough]];
      }
      case ResultTy::ros_status_successful:
      {
        return [&](auto&& res) {
          code = res->code;
          error_code_ = res->code;
          return res->values;
        }(ontologenius::compat::onto_ros::getServicePointer(res));
      }
      case ResultTy::ros_status_failure:
        [[fallthrough]];
      default:
      {
        error_code_ = -1;
        if(client_verbose)
          std::cout << COLOR_RED << "Failure to call ontologenius/" << name_ << COLOR_OFF << std::endl;
        return {"ERR:SERVICE_FAIL"};
      }
      }
    }

    /// @brief Calls the service set up in the constructor of ClientBase.
    /// @param action the query action.
    /// @param param the query parameters.
    /// @return Returns a single string. If the service call fails, the returned value is "ERR:SERVICE_FAIL".
    std::string callStr(const std::string& action, const std::string& param, int16_t& code = ignore)
    {
      auto res = this->call(action, param, code);
      return res.empty() ? "" : res[0];
    }

    /// @brief Calls the service set up in the constructor of ClientBase.
    /// @param action the query action.
    /// @param param the query parameters.
    /// @return Returns false if the service call fails.
    bool callNR(const std::string& action, const std::string& param)
    {
      return this->callStr(action, param) != "ERR:SERVICE_FAIL";
    }

    /// @brief Calls the service set up in the constructor of ClientBase.
    /// @param action the query action.
    /// @param param the query parameters.
    /// @return Returns false if the service call fails or the result code of the service is different from SUCCESS.
    bool callBool(const std::string& action, const std::string& param)
    {
      int16_t code = 0;
      auto res = this->callStr(action, param, code);

      return (res != "ERR:SERVICE_FAIL") && (code == 0);
    }

  private:
    std::string name_;
    static size_t cpt;
    static bool client_verbose;
    int error_code_;

  protected:
    ontologenius::compat::onto_ros::Client<ontologenius::compat::OntologeniusService> client_;
  };

} // namespace onto

#endif // ONTOLOGENIUS_CLIENTBASE_H