#ifndef ONTOLOGENIUS_ONTOLOGYCLIENT_H
#define ONTOLOGENIUS_ONTOLOGYCLIENT_H

#include "ontologenius/API/ontologenius/clients/ClientBase.h"

/// @brief The OntologyClient class provides an abstraction common to all ontologenius exploration ROS services.
/// The OntologyClient implements the functions common to every ontologenius exploration.
/// This class is based on ClientBase and so ensure a persistent connection with the service based on.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class OntologyClient : public ClientBase
{
public:
  /// @brief Constructs an ontology client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param n is an initialized ROS node handle.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  OntologyClient(ros::NodeHandle* n, std::string name) : ClientBase(n, name)
  {
  }

  /// @brief Gives all concepts below the one specified in the parameter.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// The default value "" represents no restriction on the result.
  std::vector<std::string> getUp(const std::string& name, int depth = -1, const std::string& selector = "");
  /// @brief Tests if a concept (individual, class, or property) inherit from another.
  /// This function corresponds to checking if class_base is part of the result of the function getUp applies to the concept.
  /// @param name is the name of the concept to be tested.
  /// @param base_class is the name of the upper concept to be compared to.
  /// @return Returns true if the concept is or inherits of the concept.
  bool isA(const std::string& name, const std::string& base_class);
  /// @brief Gives one of the label of a concept, that is not muted.
  /// @param name is the concept indentifier for which you serach a label.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @return The result of this function depends on the setting of the working language.
  std::string getName(const std::string& name, bool take_id = true);
  /// @brief Gives all the labels of a concept excepted the muted ones.
  /// @param name is the concept indentifier for which you serach its labels.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> getNames(const std::string& name, bool take_id = true);
  /// @brief Gives all the labels of a concept even the muted ones.
  /// @param name is the concept indentifier for which you serach its labels.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> getEveryNames(const std::string& name, bool take_id = true);
  /// @brief Gives all the concepts having for label the one passed in argument.
  /// @param name is a concept name in natural language.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> find(const std::string& name, bool take_id = true, const std::string& selector = "");
  /// @brief Gives all the concepts having for label a subset of the one passed in argument.
  /// @param name is a concept name in natural language.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> findSub(const std::string& name, bool take_id = true, const std::string& selector = "");
  /// @brief Gives all the concepts having a label matching the regular expression passed in argument.
  /// @param regex is a regular expression.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> findRegex(const std::string& regex, bool take_id = true, const std::string& selector = "");
  /// @brief Gives all the concepts with the lowest edit distance with the name passed in argument.
  /// @param name is a concept name in natural language.
  /// @param threshold is the minimum editing distance.
  /// This value corresponds to the number of changes to be made to pass from one
  /// word to another divided by the length of the comparison word.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> findFuzzy(const std::string& name, double threshold = 0.5, bool take_id = true, const std::string& selector = "");
  /// @brief test if a concept exist in the subpart of the ontology managed by the client
  /// (i.e. class, individuals, object properties, data properties).
  /// @param name is a concept indentifier.
  /// @return Returns true if the concept exists.
  bool exist(const std::string& name);
};

#endif // ONTOLOGENIUS_ONTOLOGYCLIENT_H
