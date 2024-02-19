#ifndef ONTOLOGENIUS_OBJECTPROPERTYCLIENT_H
#define ONTOLOGENIUS_OBJECTPROPERTYCLIENT_H

#include "ontologenius/API/ontologenius/clients/ontologyClients/OntologyClient.h"

namespace onto {

/// @brief The ObjectPropertyClient class provides an abstraction of ontologenius object properties ROS service.
/// The ontologenius object properties service allows the exploration of object properties contained by ontologenius core.
/// This class is based on ClientBase and so ensure a persistent connection with ontologenius/object_property service.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class ObjectPropertyClient : public OntologyClient
{
public:
  /// @brief Constructs an object property client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  explicit ObjectPropertyClient(const std::string& name) : OntologyClient((name == "") ? "object_property" : "object_property/" + name)
  {
  }

  /// @brief Gives all properties below the one specified in the parameter.
  /// @param name is an object property identifier.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of object properties.
  std::vector<std::string> getDown(const std::string& name, int depth = -1);
  /// @brief Gives all the disjoint properties of the one specified in the parameter.
  /// @param name is an object property identifier.
  /// @return a vector of identifiers of object properties.
  std::vector<std::string> getDisjoint(const std::string& name);
  /// @brief Gives all the domain classes of the object property specified in the parameter.
  /// @param name is an object property identifier.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation while the value 0
  /// corresponds to the direct domains
  /// @return a vector of identifiers of classes.
  std::vector<std::string> getDomain(const std::string& name, int depth = -1);
  /// @brief Gives all the ranges classes of the object property specified in the parameter.
  /// @param name is an object property identifier.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation while the value 0
  /// corresponds to the direct ranges
  /// @return a vector of identifiers of classes.
  std::vector<std::string> getRange(const std::string& name, int depth = -1);
  /// @brief Gives all the inverses properties of the one specified in the parameter.
  /// @param name is an object property identifier.
  /// @return a vector of identifiers of object properties.
  std::vector<std::string> getInverse(const std::string& name);

private:

};

} // namespace onto

#endif // ONTOLOGENIUS_OBJECTPROPERTYCLIENT_H
