#ifndef ONTOLOGENIUS_DATAPROPERTYCLIENT_H
#define ONTOLOGENIUS_DATAPROPERTYCLIENT_H

#include "ontologenius/API/ontologenius/clients/ontologyClients/OntologyClient.h"

namespace onto {

/// @brief The DataPropertyClient class provides an abstraction of ontologenius data properties ROS service.
/// The ontologenius data properties service allows the exploration of data properties contained by ontologenius core.
/// This class is based on ClientBase and so ensure a persistent connection with ontologenius/data_property service.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class DataPropertyClient : public OntologyClient
{
public:
  /// @brief Constructs a data property client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  explicit DataPropertyClient(const std::string& name) : OntologyClient((name == "") ? "data_property" : "data_property/" + name)
  {
  }

  /// @brief Gives all properties below the one specified in the parameter.
  /// @param name is a data property identifier.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of data properties.
  std::vector<std::string> getDown(const std::string& name, int depth = -1);
  /// @brief Gives all properties disjoint of the one specified in the parameter.
  /// @param name is a data property identifier.
  /// @return a vector of identifiers of data properties.
  std::vector<std::string> getDisjoint(const std::string& name);
  /// @brief Gives all the domain classes of the specified property.
  /// @param name is a data property identifier.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation while the value 0
  /// corresponds to the direct domains
  /// @return a vector of identifiers of classes.
  std::vector<std::string> getDomain(const std::string& name, int depth = -1);
  /// @brief Gives all the range classes of the specified property.
  /// @param name is a data property identifier.
  /// @return a vector of identifiers of classes.
  std::vector<std::string> getRange(const std::string& name);

private:

};

} // namespace onto

#endif // ONTOLOGENIUS_DATAPROPERTYCLIENT_H
