#ifndef ONTOLOGENIUS_SPARQLINDEXCLIENT_H
#define ONTOLOGENIUS_SPARQLINDEXCLIENT_H

#include <ros/ros.h>

#include "ontologenius/OntologeniusSparqlIndexService.h"
#include "ontologenius/OntologeniusSparqlIndexResponse.h"

namespace onto {

/// @brief The SparqlIndexClient class provides a ROS service to explore ontologenius with SPARQL-like queries based on indexes.
/// The variables start with the symbol ? (e.g. ?my_var) and each triplet is separated by a comma.
class SparqlIndexClient
{
public:
  /// @brief Constructs a sparql client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param n is an initialized ROS node handle.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  SparqlIndexClient(ros::NodeHandle* n, const std::string& name) : client(n->serviceClient<ontologenius::OntologeniusSparqlIndexService>((name == "") ? "ontologenius/sparql_index" : "ontologenius/sparql_index/" + name, true)),
                                                              name_((name == "") ? "sparql_index" : "sparql_index/" + name)
  {
    n_ = n;
  }

  std::vector<ontologenius::OntologeniusSparqlIndexResponse> call(const std::string& query);

private:
  ros::ServiceClient client;
  std::string name_;
  ros::NodeHandle* n_;
};

} // namespace onto

#endif // ONTOLOGENIUS_SPARQLINDEXCLIENT_H
