#include "ontologenius/API/ontologenius/clientsIndex/SparqlIndexClient.h"

namespace onto {

std::pair<std::vector<std::string>, std::vector<ontologenius::OntologeniusSparqlIndexResponse>> SparqlIndexClient::call(const std::string& query)
{
  ontologenius::OntologeniusSparqlIndexService srv;
  srv.request.query = query;

  if(client.call(srv))
    return {srv.response.names, srv.response.results};
  else
  {
    client = n_.serviceClient<ontologenius::OntologeniusSparqlIndexService>("ontologenius/" + name_, true);
    if(client.call(srv))
      return {srv.response.names, srv.response.results};
    else
      return {};
  }
}

} // namespace onto
