#include "ontologenius/API/ontologenius/clients/SparqlClient.h"

namespace onto {

std::vector<ontologenius::OntologeniusSparqlResponse> SparqlClient::call(const std::string& query)
{
  ontologenius::OntologeniusSparqlService srv;
  srv.request.query = query;

  if(client.call(srv))
    return srv.response.results;
  else
  {
    client = n_->serviceClient<ontologenius::OntologeniusSparqlService>("ontologenius/" + name_, true);
    if(client.call(srv))
      return srv.response.results;
    else
      return {};
  }
}

} // namespace onto