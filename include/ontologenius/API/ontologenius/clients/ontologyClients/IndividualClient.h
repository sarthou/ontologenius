#ifndef ONTOLOGENIUS_INDIVIDUALCLIENT_H
#define ONTOLOGENIUS_INDIVIDUALCLIENT_H

#include "ontologenius/API/ontologenius/clients/ontologyClients/OntologyClient.h"

namespace onto {

/// @brief The IndividualClient class provides an abstraction of ontologenius individuals ROS service.
/// The ontologenius individuals' service allows the exploration of individuals contained by ontologenius core.
/// This class is based on ClientBase and so ensure a persistent connection with ontologenius/individual service.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails.
class IndividualClient : public OntologyClient
{
public:
  /// @brief Constructs an individual client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  explicit IndividualClient(const std::string& name) : OntologyClient((name == "")? "individual" : "individual/" + name)
  {
  }

  /// @brief Gives all the individuals pointed by the specified property and applied to the specified individual.
  /// @param name is the identifier of the individual being the subject of the triplet.
  /// @param property is the identifier of the property being the predicat of the triplet.
  /// @param selector can be set to only get results inheriting from the selector class.
  /// The default value "" represents no restriction on the result.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getOn(const std::string& name, const std::string& property, const std::string& selector = "");
  /// @brief Gives all the individuals having the specified property pointing to the specified individual.
  /// @param property is the identifier of the property being the predicat of the triplet.
  /// @param name is the identifier of the individual being the object of the triplet.
  /// @param selector can be set to only get results inheriting from the selector class.
  /// The default value "" represents no restriction on the result.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getFrom(const std::string& property, const std::string& name, const std::string& selector = "");
  /// @brief Gives all the properties linking the two specified individuals.
  /// @param indiv_from is the identifier of the individual being the subject of the triplet.
  /// @param indiv_to is the identifier of the individual being the object of the triplet.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value "" represents no restriction on the result.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<std::string> getWith(const std::string& indiv_from, const std::string& indiv_to, const std::string& selector = "", int depth = -1);

  /// @brief Gives all the individuals possessing the specified property.
  /// @param name is the identifier of the property being the predicat of the triplet.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getRelatedFrom(const std::string& property);
  /// @brief Gives all the individuals pointed to by the specified property.
  /// @param name is the identifier of the property being the predicat of the triplet.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getRelatedOn(const std::string& property);
  /// @brief Gives all the individuals having a property pointing to the specified individual.
  /// @param name is the identifier of the individual being the subject of the triplet.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getRelatedWith(const std::string& name);

  /// @brief Gives all the properties applied to the specified individual.
  /// @param name is the identifier of the individual being the subject of the triplet.
  /// @return a vector of identifiers of properties.
  std::vector<std::string> getRelationFrom(const std::string& name, int depth = -1);
  /// @brief Gives all the properties pointing to the specified individual.
  /// @param name is the identifier of the individual being the object of the triplet.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<std::string> getRelationOn(const std::string& name, int depth = -1);
  /// @brief Gives all the individuals pointed by a property applied to the specified individual.
  /// @param name is the identifier of the individual being the object of the triplet.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getRelationWith(const std::string& name);

  /// @brief Gives all the properties for which the specified individual is part of the domain.
  /// @param name is the identifier of an individual.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value "" represents no restriction on the result.
  /// @param depth can be set to limit tree propagation of the individual to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of properties.
  std::vector<std::string> getDomainOf(const std::string& name, const std::string& selector = "", int depth = -1);
  /// @brief Gives all the properties for which the specified individual is part of the range.
  /// @param name is the identifier of an individual.
  /// @param selector can be set to only get results inheriting from the selector property.
  /// The default value "" represents no restriction on the result.
  /// @param depth can be set to limit tree propagation of the individual to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @return a vector of identifiers of object properties.
  std::vector<std::string> getRangeOf(const std::string& name, const std::string& selector = "", int depth = -1);

  /// @brief Gives all the individuals of the type of the class specified in the parameter.
  /// @param name is a class identifier.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getType(const std::string& name);
  /// @brief Gives all the individuals that are defined as being identical to the individual specified in the parameter.
  /// @param name is an individual identifier.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getSame(const std::string& name);
  /// @brief Gives all the individuals that are defined as being distinct to the individual specified in the parameter.
  /// @param name is an individual identifier.
  /// @return a vector of identifiers of individuals.
  std::vector<std::string> getDistincts(const std::string& name);

private:

};

} // namespace onto

#endif // ONTOLOGENIUS_INDIVIDUALCLIENT_H
