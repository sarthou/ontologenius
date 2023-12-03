#ifndef ONTOLOGENIUS_CLASSINDEXCLIENT_H
#define ONTOLOGENIUS_CLASSINDEXCLIENT_H

#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/OntologyIndexClient.h"

namespace onto {

/// @brief The ClassIndexClient class provides an abstraction of ontologenius classes ROS service.
/// The ontologenius classes service allows the exploration of classes contained by ontologenius core.
/// This class is based on ClientBase and so ensure a persistent connection with ontologenius/class service.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class ClassIndexClient : public OntologyIndexClient
{
public:
  /// @brief Constructs a class client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  ClassIndexClient(const std::string& name) : OntologyIndexClient((name == "") ? "class_index" : "class_index/" + name)
  {
  }

  /// @brief Gives all classes below the one given in the specified parameter.
  /// @param index is a class identifier.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @param selector can be set to only get results inheriting from the selector class.
  /// The default value 0 represents no restriction on the result.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getDown(int64_t index, int depth = -1, int64_t selector = 0);
  /// @brief Gives all the disjoint classes of the one specified in the parameter.
  /// @param index is a class identifier.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getDisjoint(int64_t index);

  /// @brief Gives all the classes pointed by the specified property and applied to the specified class.
  /// @param index is the identifier of the class being the subject of the triplet.
  /// @param property is the identifier of the property being the predicat of the triplet.
  /// @param selector can be set to only get results inheriting from the selector class.
  /// The default value 0 represents no restriction on the result.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getOn(int64_t index, int64_t property, int64_t selector = 0);
  /// @brief Gives all the classes having the specified property pointing to the specified class.
  /// @param property is the identifier of the property being the predicat of the triplet.
  /// @param index is the identifier of the class being the object of the triplet.
  /// @param selector can be set to only get results inheriting from the selector class.
  /// The default value 0 represents no restriction on the result.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getFrom(int64_t property, int64_t index, int64_t selector = 0);
  /// @brief Gives all the properties linking the two specified classes.
  /// @param indiv_from is the identifier of the class being the subject of the triplet.
  /// @param indiv_to is the identifier of the class being the object of the triplet.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value 0 represents no restriction on the result.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<int64_t> getWith(int64_t class_from, int64_t class_to, int64_t selector = 0, int depth = -1);

  /// @brief Gives all the classes possessing the specified property.
  /// @param index is the identifier of the property being the predicat of the triplet.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getRelatedFrom(int64_t property);
  /// @brief Gives all the classes pointed to by the specified property.
  /// @param index is the identifier of the property being the predicat of the triplet.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getRelatedOn(int64_t property);
  /// @brief Gives all the classes having a property pointing to the specified class.
  /// @param index is the identifier of the class being the subject of the triplet.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getRelatedWith(int64_t index);

  /// @brief Gives all the properties applied to the specified class.
  /// @param index is the identifier of the class being the subject of the triplet.
  /// @return a vector of identifiers of properties.
  std::vector<int64_t> getRelationFrom(int64_t index, int depth = -1);
  /// @brief Gives all the properties pointing to the specified class.
  /// @param index is the identifier of the class being the object of the triplet.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<int64_t> getRelationOn(int64_t index, int depth = -1);
  /// @brief Gives all the classes pointed by a property applied to the specified class.
  /// @param index is the identifier of the class being the object of the triplet.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of classes.
  std::vector<int64_t> getRelationWith(int64_t index);

  /// @brief Gives all the properties for which the specified class is part of the domain.
  /// @param index is the identifier of an class.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value 0 represents no restriction on the result.
  /// @param depth can be set to limit tree propagation of the class to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<int64_t> getDomainOf(int64_t index, int64_t selector = 0, int depth = -1);
  /// @brief Gives all the properties for which the specified class is part of the range.
  /// @param index is the identifier of an class.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value 0 represents no restriction on the result.
  /// @param depth can be set to limit tree propagation of the class to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of object properties.
  std::vector<int64_t> getRangeOf(int64_t index, int64_t selector = 0, int depth = -1);

private:

};

} // namepsace onto

#endif // ONTOLOGENIUS_CLASSINDEXCLIENT_H
