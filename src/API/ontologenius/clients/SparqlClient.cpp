#include "ontologenius/API/ontologenius/clients/SparqlClient.h"

namespace onto {

std::pair<std::vector<std::string>, std::vector<ontologenius::compat::OntologeniusSparqlResponse>> SparqlClient::call(const std::string& query)
{
  auto req = ontologenius::compat::make_request<ontologenius::compat::OntologeniusSparqlService>();
  auto res = ontologenius::compat::make_response<ontologenius::compat::OntologeniusSparqlService>();

  [query](auto&& req)
  {
    req->query = query;
  }(ontologenius::compat::onto_ros::getServicePointer(req));

  using ResultTy = typename decltype(client_)::Status;

  switch (client_.call(req, res))
  {
    case ResultTy::SUCCESSFUL_WITH_RETRIES:
      [[fallthrough]];
    case ResultTy::SUCCESSFUL:
    {
      return [](auto&& res) -> std::pair<std::vector<std::string>, std::vector<ontologenius::compat::OntologeniusSparqlResponse>>
      {
        return { res->names, res->results };
      }(ontologenius::compat::onto_ros::getServicePointer(res));
        
    }
    case ResultTy::FAILURE:
      [[fallthrough]];
    default:
    {
      return { };
    }
  }
}

} // namespace onto