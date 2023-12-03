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
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  SparqlIndexClient(const std::string& name) : name_((name == "") ? "sparql_index" : "sparql_index/" + name),
                                               client(n_.serviceClient<ontologenius::OntologeniusSparqlIndexService>((name == "") ? "ontologenius/sparql_index" : "ontologenius/sparql_index/" + name, true))
                                                              
  {}

  std::pair<std::vector<std::string>, std::vector<ontologenius::OntologeniusSparqlIndexResponse>> call(const std::string& query);

private:
  ros::NodeHandle n_;
  std::string name_;
  ros::ServiceClient client;
};

} // namespace onto

#endif // ONTOLOGENIUS_SPARQLINDEXCLIENT_H
