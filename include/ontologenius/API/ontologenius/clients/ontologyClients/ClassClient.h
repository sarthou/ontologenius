#ifndef ONTOLOGENIUS_CLASSCLIENT_H
#define ONTOLOGENIUS_CLASSCLIENT_H

#include "ontologenius/API/ontologenius/clients/ontologyClients/OntologyClient.h"

namespace onto {

  /// @brief The ClassClient class provides an abstraction of ontologenius classes ROS service.
  /// The ontologenius classes service allows the exploration of classes contained by ontologenius core.
  /// This class is based on ClientBase and so ensure a persistent connection with ontologenius/class service.
  /// The persistent connection ensures a minimal response time.
  /// A reconnection logic is implemented in the event that the persistent connection fails.
  class ClassClient : public OntologyClient
  {
  public:
    /// @brief Constructs a class client.
    /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
    /// @param name is the instance to be connected to. For classic use, name should be defined as "".
    explicit ClassClient(const std::string& name) : OntologyClient((name.empty()) ? "class" : "class/" + name)
    {}

    /// @brief Gives all classes below the one given in the specified parameter.
    /// @param name is a class identifier.
    /// @param depth can be set to limit tree propagation to a specific value.
    /// The default value -1 represents no propagation limitation.
    /// @param selector can be set to only get results inheriting from the selector class.
    /// The default value "" represents no restriction on the result.
    /// @return a vector of identifiers of classes.
    std::vector<std::string> getDown(const std::string& name, int depth = -1, const std::string& selector = "");
    /// @brief Gives all the disjoint classes of the one specified in the parameter.
    /// @param name is a class identifier.
    /// @return a vector of identifiers of classes.
    std::vector<std::string> getDisjoint(const std::string& name);

    /// @brief Gives all the classes pointed by the specified property and applied to the specified class.
    /// @param name is the identifier of the class being the subject of the triplet.
    /// @param property is the identifier of the property being the predicat of the triplet.
    /// @param selector can be set to only get results inheriting from the selector class.
    /// The default value "" represents no restriction on the result.
    /// @return a vector of identifiers of classes.
    std::vector<std::string> getOn(const std::string& name, const std::string& property, const std::string& selector = "");
    /// @brief Gives all the classes having the specified property pointing to the specified class.
    /// @param property is the identifier of the property being the predicat of the triplet.
    /// @param name is the identifier of the class being the object of the triplet.
    /// @param selector can be set to only get results inheriting from the selector class.
    /// The default value "" represents no restriction on the result.
    /// @return a vector of identifiers of classes.
    std::vector<std::string> getFrom(const std::string& property, const std::string& name, const std::string& selector = "");
    /// @brief Gives all the properties linking the two specified classes.
    /// @param indiv_from is the identifier of the class being the subject of the triplet.
    /// @param indiv_to is the identifier of the class being the object of the triplet.
    /// @param selector can be set to only get results inheriting from the selector property.
    /// The default value "" represents no restriction on the result.
    /// @param depth can be set to limit tree propagation to a specific value.
    /// The default value -1 represents no propagation limitation.
    /// @return a vector of identifiers of properties.
    std::vector<std::string> getWith(const std::string& class_from, const std::string& class_to, const std::string& selector = "", int depth = -1);

    /// @brief Gives all the classes possessing the specified property.
    /// @param name is the identifier of the property being the predicat of the triplet.
    /// @return a vector of identifiers of classes.
    std::vector<std::string> getRelatedFrom(const std::string& property);
    /// @brief Gives all the classes pointed to by the specified property.
    /// @param name is the identifier of the property being the predicat of the triplet.
    /// @return a vector of identifiers of classes.
    std::vector<std::string> getRelatedOn(const std::string& property);
    /// @brief Gives all the classes having a property pointing to the specified class.
    /// @param name is the identifier of the class being the subject of the triplet.
    /// @return a vector of identifiers of classes.
    std::vector<std::string> getRelatedWith(const std::string& name);

    /// @brief Gives all the properties applied to the specified class.
    /// @param name is the identifier of the class being the subject of the triplet.
    /// @return a vector of identifiers of properties.
    std::vector<std::string> getRelationFrom(const std::string& name, int depth = -1);
    /// @brief Gives all the properties pointing to the specified class.
    /// @param name is the identifier of the class being the object of the triplet.
    /// @param depth can be set to limit tree propagation to a specific value.
    /// The default value -1 represents no propagation limitation.
    /// @return a vector of identifiers of properties.
    std::vector<std::string> getRelationOn(const std::string& name, int depth = -1);
    /// @brief Gives all the classes pointed by a property applied to the specified class.
    /// @param name is the identifier of the class being the object of the triplet.
    /// @param depth can be set to limit tree propagation to a specific value.
    /// The default value -1 represents no propagation limitation.
    /// @return a vector of identifiers of classes.
    std::vector<std::string> getRelationWith(const std::string& name);

    /// @brief Gives all the properties for which the specified class is part of the domain.
    /// @param name is the identifier of an class.
    /// @param selector can be set to only get results inheriting from the selector property.
    /// The default value "" represents no restriction on the result.
    /// @param depth can be set to limit tree propagation of the class to a specific value.
    /// The default value -1 represents no propagation limitation.
    /// @return a vector of identifiers of properties.
    std::vector<std::string> getDomainOf(const std::string& name, const std::string& selector = "", int depth = -1);
    /// @brief Gives all the properties for which the specified class is part of the range.
    /// @param name is the identifier of an class.
    /// @param selector can be set to only get results inheriting from the selector property.
    /// The default value "" represents no restriction on the result.
    /// @param depth can be set to limit tree propagation of the class to a specific value.
    /// The default value -1 represents no propagation limitation.
    /// @return a vector of identifiers of object properties.
    std::vector<std::string> getRangeOf(const std::string& name, const std::string& selector = "", int depth = -1);

  private:
  };

} // namespace onto

#endif // ONTOLOGENIUS_CLASSCLIENT_H
