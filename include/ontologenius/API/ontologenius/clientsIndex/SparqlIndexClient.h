#ifndef ONTOLOGENIUS_SPARQLINDEXCLIENT_H
#define ONTOLOGENIUS_SPARQLINDEXCLIENT_H

#include "ontologenius/compat/ros.h"

namespace onto {

/// @brief The SparqlIndexClient class provides a ROS service to explore ontologenius with SPARQL-like queries based on indexes.
/// The variables start with the symbol ? (e.g. ?my_var) and each triplet is separated by a comma.
class SparqlIndexClient
{
public:
  /// @brief Constructs a sparql client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  explicit SparqlIndexClient(const std::string& name) : client_((name == "") ? "/ontologenius/sparql_index" : "/ontologenius/sparql_index/" + name)
                                                              
  {}

  std::pair<std::vector<std::string>, std::vector<ontologenius::compat::OntologeniusSparqlIndexResponse>> call(const std::string& query);

private:
  ontologenius::compat::onto_ros::Client<ontologenius::compat::OntologeniusSparqlIndexService> client_;
};

} // namespace onto

#endif // ONTOLOGENIUS_SPARQLINDEXCLIENT_H
