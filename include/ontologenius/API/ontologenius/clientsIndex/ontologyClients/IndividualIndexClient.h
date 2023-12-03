#ifndef ONTOLOGENIUS_INDIVIDUALINDEXCLIENT_H
#define ONTOLOGENIUS_INDIVIDUALINDEXCLIENT_H

#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/OntologyIndexClient.h"

namespace onto {

/// @brief The IndividualIndexClient class provides an abstraction of ontologenius individuals ROS service.
/// The ontologenius individuals' service allows the exploration of individuals contained by ontologenius core.
/// This class is based on ClientBase and so ensure a persistent connection with ontologenius/individual service.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails.
class IndividualIndexClient : public OntologyIndexClient
{
public:
  /// @brief Constructs an individual client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  IndividualIndexClient(const std::string& name) : OntologyIndexClient((name == "")? "individual_index" : "individual_index/" + name)
  {
  }

  /// @brief Gives all the individuals pointed by the specified property and applied to the specified individual.
  /// @param index is the identifier of the individual being the subject of the triplet.
  /// @param property is the identifier of the property being the predicat of the triplet.
  /// @param selector can be set to only get results inheriting from the selector class.
  /// The default value "" represents no restriction on the result.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getOn(int64_t index, int64_t property, int64_t selector = 0);
  /// @brief Gives all the individuals having the specified property pointing to the specified individual.
  /// @param property is the identifier of the property being the predicat of the triplet.
  /// @param index is the identifier of the individual being the object of the triplet.
  /// @param selector can be set to only get results inheriting from the selector class.
  /// The default value "" represents no restriction on the result.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getFrom(int64_t property, int64_t index, int64_t selector = 0);
  /// @brief Gives all the properties linking the two specified individuals.
  /// @param indiv_from is the identifier of the individual being the subject of the triplet.
  /// @param indiv_to is the identifier of the individual being the object of the triplet.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value "" represents no restriction on the result.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<int64_t> getWith(int64_t indiv_from, int64_t indiv_to, int64_t selector = 0, int depth = -1);

  /// @brief Gives all the individuals possessing the specified property.
  /// @param index is the identifier of the property being the predicat of the triplet.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getRelatedFrom(int64_t property);
  /// @brief Gives all the individuals pointed to by the specified property.
  /// @param index is the identifier of the property being the predicat of the triplet.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getRelatedOn(int64_t property);
  /// @brief Gives all the individuals having a property pointing to the specified individual.
  /// @param index is the identifier of the individual being the subject of the triplet.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getRelatedWith(int64_t index);

  /// @brief Gives all the properties applied to the specified individual.
  /// @param index is the identifier of the individual being the subject of the triplet.
  /// @return a vector of identifiers of properties.
  std::vector<int64_t> getRelationFrom(int64_t index, int depth = -1);
  /// @brief Gives all the properties pointing to the specified individual.
  /// @param index is the identifier of the individual being the object of the triplet.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<int64_t> getRelationOn(int64_t index, int depth = -1);
  /// @brief Gives all the individuals pointed by a property applied to the specified individual.
  /// @param index is the identifier of the individual being the object of the triplet.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getRelationWith(int64_t index);

  /// @brief Gives all the properties for which the specified individual is part of the domain.
  /// @param index is the identifier of an individual.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value "" represents no restriction on the result.
  /// @param depth can be set to limit tree propagation of the individual to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<int64_t> getDomainOf(int64_t index, int64_t selector = 0, int depth = -1);
  /// @brief Gives all the properties for which the specified individual is part of the range.
  /// @param index is the identifier of an individual.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value "" represents no restriction on the result.
  /// @param depth can be set to limit tree propagation of the individual to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of object properties.
  std::vector<int64_t> getRangeOf(int64_t index, int64_t selector = 0, int depth = -1);

  /// @brief Gives all the individuals of the type of the class specified in the parameter.
  /// @param index is a class identifier.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getType(int64_t index);
  /// @brief Gives all the individuals that are defined as being identical to the individual specified in the parameter.
  /// @param index is an individual identifier.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getSame(int64_t index);
  /// @brief Gives all the individuals that are defined as being distinct to the individual specified in the parameter.
  /// @param index is an individual identifier.
  /// @return a vector of identifiers of individuals.
  std::vector<int64_t> getDistincts(int64_t index);

private:

};

} // namespace onto

#endif // ONTOLOGENIUS_INDIVIDUALINDEXCLIENT_H
