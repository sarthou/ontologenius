#ifndef ONTOLOGENIUS_SPARQLCLIENT_H
#define ONTOLOGENIUS_SPARQLCLIENT_H

#include <ros/ros.h>

#include "ontologenius/OntologeniusSparqlService.h"
#include "ontologenius/OntologeniusSparqlResponse.h"

namespace onto {

/// @brief The SparqlClient class provides a ROS service to explore ontologenius with SPARQL-like queries.
/// The variables start with the symbol ? (e.g. ?my_var) and each triplet is separated by a comma.
class SparqlClient
{
public:
  /// @brief Constructs a sparql client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  SparqlClient(const std::string& name) : client(n_.serviceClient<ontologenius::OntologeniusSparqlService>((name == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name, true)),
                                          name_((name == "") ? "sparql" : "sparql/" + name)
  {}

  std::pair<std::vector<std::string>, std::vector<ontologenius::OntologeniusSparqlResponse>> call(const std::string& query);

private:
  ros::NodeHandle n_;
  ros::ServiceClient client;
  std::string name_;
};

} // namespace onto

#endif // ONTOLOGENIUS_SPARQLCLIENT_H
