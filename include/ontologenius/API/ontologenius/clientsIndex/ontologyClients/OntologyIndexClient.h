#ifndef ONTOLOGENIUS_ONTOLOGYINDEXCLIENT_H
#define ONTOLOGENIUS_ONTOLOGYINDEXCLIENT_H

#include "ontologenius/API/ontologenius/clientsIndex/ClientBaseIndex.h"

namespace onto {

/// @brief The OntologyIndexClient class provides an abstraction common to all ontologenius exploration ROS services based on indexes.
/// The OntologyIndexClient implements the functions common to every ontologenius exploration.
/// This class is based on ClientBaseIndex and so ensure a persistent connection with the service based on.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class OntologyIndexClient : public ClientBaseIndex
{
public:
  /// @brief Constructs an ontology client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  OntologyIndexClient(const std::string& name) : ClientBaseIndex(name)
  {
  }

  /// @brief Gives all concepts below the one specified in the parameter.
  /// @param depth can be set to limit tree propagation to a specific value.
  /// The default value -1 represents no propagation limitation.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// The default value 0 represents no restriction on the result.
  std::vector<int64_t> getUp(int64_t index, int depth = -1, int64_t selector = 0);
  /// @brief Tests if a concept (individual, class, or property) inherit from another.
  /// This function corresponds to checking if class_base is part of the result of the function getUp applies to the concept.
  /// @param index is the index of the concept to be tested.
  /// @param base_class is the index of the upper concept to be compared to.
  /// @return Returns true if the concept is or inherits of the concept.
  bool isA(int64_t index, int64_t base_class);
  /// @brief Gives one of the label of a concept, that is not muted.
  /// @param index is the concept indentifier for which you serach a label.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @return The result of this function depends on the setting of the working language.
  std::string getName(int64_t index, bool take_id = true);
  /// @brief Gives all the labels of a concept excepted the muted ones.
  /// @param index is the concept indentifier for which you serach its labels.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> getNames(int64_t index, bool take_id = true);
  /// @brief Gives all the labels of a concept even the muted ones.
  /// @param index is the concept indentifier for which you serach its labels.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> getEveryNames(int64_t index, bool take_id = true);
  /// @brief Gives all the concepts having for label the one passed in argument.
  /// @param name is a concept name in natural language.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<int64_t> find(const std::string& name, bool take_id = true, int64_t selector = 0);
  /// @brief Gives all the concepts having for label a subset of the one passed in argument.
  /// @param name is a concept name in natural language.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<int64_t> findSub(const std::string& name, bool take_id = true, int64_t selector = 0);
  /// @brief Gives all the concepts having a label matching the regular expression passed in argument.
  /// @param regex is a regular expression.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<int64_t> findRegex(const std::string& regex, bool take_id = true, int64_t selector = 0);
  /// @brief Gives all the names of concepts with the lowest edit distance with the name passed in argument.
  /// @param name is a concept name in natural language.
  /// @param threshold is the minimum editing distance.
  /// This value corresponds to the number of changes to be made to pass from one
  /// word to another divided by the length of the comparison word.
  /// @param take_id can be set to false if you do not want to consider the concept identifier as a possible default name.
  /// @param selector can be set to only get results inheriting from the selector concept.
  /// @return The result of this function depends on the setting of the working language.
  std::vector<std::string> findFuzzy(const std::string& name, double threshold = 0.5, bool take_id = true, int64_t selector = 0);
  /// @brief test if a concept exist in the subpart of the ontology managed by the client
  /// (i.e. class, individuals, object properties, data properties).
  /// @param index is a concept indentifier as an integer.
  /// @return Returns true if the concept exists.
  bool exist(int64_t index);
};

} // namespace onto

#endif // ONTOLOGENIUS_ONTOLOGYINDEXCLIENT_H
