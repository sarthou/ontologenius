#ifndef ONTOLOGENIUS_SPARQLCLIENT_H
#define ONTOLOGENIUS_SPARQLCLIENT_H

#include "ontologenius/compat/ros.h"

namespace onto {

class OntologyManipulator;
class OntologyManipulatorIndex;

/// @brief The SparqlClient class provides a ROS service to explore ontologenius with SPARQL-like queries.
/// The variables start with the symbol ? (e.g. ?my_var) and each triplet is separated by a comma.
class SparqlClient
{
  friend OntologyManipulator;
  friend OntologyManipulatorIndex;
public:
  /// @brief Constructs a sparql client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  explicit SparqlClient(const std::string& name) : client_((name == "") ? "/ontologenius/sparql" : "/ontologenius/sparql/" + name)  {}

  std::pair<std::vector<std::string>, std::vector<ontologenius::compat::OntologeniusSparqlResponse>> call(const std::string& query);
private:
  ontologenius::compat::onto_ros::Client<ontologenius::compat::OntologeniusSparqlService> client_;
};

} // namespace onto

#endif // ONTOLOGENIUS_SPARQLCLIENT_H
