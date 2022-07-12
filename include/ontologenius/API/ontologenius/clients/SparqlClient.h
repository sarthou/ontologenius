#ifndef ONTOLOGENIUS_SPARQLCLIENT_H
#define ONTOLOGENIUS_SPARQLCLIENT_H

#include <ros/ros.h>

#include "ontologenius/OntologeniusSparqlService.h"
#include "ontologenius/OntologeniusSparqlResponse.h"

/// @brief The SparqlClient class provides a ROS service to explore ontologenius with SPARQL-like queries.
/// The variables start with the symbol ? (e.g. ?my_var) and each triplet is separated by a comma.
class SparqlClient
{
public:
  /// @brief Constructs a sparql client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param n is an initialized ROS node handle.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  SparqlClient(ros::NodeHandle* n, const std::string& name) : client(n->serviceClient<ontologenius::OntologeniusSparqlService>((name == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name, true)),
                                                              name_((name == "") ? "sparql" : "sparql/" + name)
  {
    n_ = n;
  }

  std::vector<ontologenius::OntologeniusSparqlResponse> call(const std::string& query);

private:
  ros::ServiceClient client;
  std::string name_;
  ros::NodeHandle* n_;
};

#endif // ONTOLOGENIUS_SPARQLCLIENT_H
