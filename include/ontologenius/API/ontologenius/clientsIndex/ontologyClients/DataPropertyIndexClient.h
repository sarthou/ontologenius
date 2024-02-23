#ifndef ONTOLOGENIUS_DATAPROPERTYINDEXCLIENT_H
#define ONTOLOGENIUS_DATAPROPERTYINDEXCLIENT_H

#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/OntologyIndexClient.h"

namespace onto {

/// @brief The DataPropertyIndexClient class provides an abstraction of ontologenius data properties ROS service.
/// The ontologenius data properties service allows the exploration of data properties contained by ontologenius core.
/// This class is based on ClientBase and so ensure a persistent connection with ontologenius/data_property service.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class DataPropertyIndexClient : public OntologyIndexClient
{
public:
  /// @brief Constructs a data property client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  explicit DataPropertyIndexClient(const std::string& name) : OntologyIndexClient((name == "") ? "data_property_index" : "data_property_index/" + name)
  {
  }

  /// @brief Gives all properties below the one specified in the parameter.
  /// @param index is a data property identifier.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of data properties.
  std::vector<int64_t> getDown(int64_t index, int depth = -1);
  /// @brief Gives all properties disjoint of the one specified in the parameter.
  /// @param index is a data property identifier.
  /// @return a vector of identifiers of data properties.
  std::vector<int64_t> getDisjoint(int64_t index);
  /// @brief Gives all the domain classes of the specified property.
  /// @param index is a data property identifier.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation while the value 0
  /// corresponds to the direct domains
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getDomain(int64_t index, int depth = -1);
  /// @brief Gives all the range classes of the specified property.
  /// @param index is a data property identifier.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getRange(int64_t index);

private:

};

} // namespace onto

#endif // ONTOLOGENIUS_DATAPROPERTYINDEXCLIENT_H
