#include "ontologenius/API/ontologenius/clients/SparqlClient.h"

#include <string>
#include <utility>
#include <vector>

#include "ontologenius/compat/ros.h"

namespace onto {

  std::pair<std::vector<std::string>, std::vector<ontologenius::compat::OntologeniusSparqlResponse>> SparqlClient::call(const std::string& query)
  {
    auto req = ontologenius::compat::makeRequest<ontologenius::compat::OntologeniusSparqlService>();
    auto res = ontologenius::compat::makeResponse<ontologenius::compat::OntologeniusSparqlService>();

    [query](auto&& req) {
      req->query = query;
    }(ontologenius::compat::onto_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_)::Status_e;

    switch(client_.call(req, res))
    {
    case ResultTy::ros_status_successful_with_retry:
      [[fallthrough]];
    case ResultTy::ros_status_successful:
    {
      return [](auto&& res) -> std::pair<std::vector<std::string>, std::vector<ontologenius::compat::OntologeniusSparqlResponse>> {
        return {res->names, res->results};
      }(ontologenius::compat::onto_ros::getServicePointer(res));
    }
    case ResultTy::ros_status_failure:
      [[fallthrough]];
    default:
    {
      return {};
    }
    }
  }

} // namespace onto