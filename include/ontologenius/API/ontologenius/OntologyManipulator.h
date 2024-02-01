#ifndef ONTOLOGENIUS_ONTOLOGYMANIPULATOR_H
#define ONTOLOGENIUS_ONTOLOGYMANIPULATOR_H

#include <vector>
#include <string>

#include "ontologenius/API/ontologenius/clients/ontologyClients/IndividualClient.h"
#include "ontologenius/API/ontologenius/clients/ontologyClients/ObjectPropertyClient.h"
#include "ontologenius/API/ontologenius/clients/ontologyClients/DataPropertyClient.h"
#include "ontologenius/API/ontologenius/clients/ontologyClients/ClassClient.h"
#include "ontologenius/API/ontologenius/clients/SparqlClient.h"
#include "ontologenius/API/ontologenius/clients/ActionClient.h"
#include "ontologenius/API/ontologenius/clients/ReasonerClient.h"
#include "ontologenius/API/ontologenius/FeederPublisher.h"

namespace onto {

/// @brief The OntologyManipulator class is just an object to access all API ROS abstraction classes so that you can query and manage ontologenius.
class OntologyManipulator
{
private:
  std::string name_;

public:
  /// @brief Constructs an ontology manipulator.Can be used in a multi-ontology mode by specifying the name of the ontology name. For classic use, do not specify the ontology name name.
  /// @param name is the instance name
  OntologyManipulator(const std::string& name = "");
  /// @brief OntologyManipulator copy constructor
  /// @param other a reference to the object to copy
  OntologyManipulator(const OntologyManipulator& other);
  OntologyManipulator(OntologyManipulator&& other);
  /// @brief OntologyManipulator desconstructor
  ~OntologyManipulator() {}

  /// @brief Gives the total number of service calls from all ROS clients instances since the last reset
  size_t nb() {return actions.nb();}
  /// @brief Reset the call counter for all instances of ROS clients
  void resetNb() {actions.resetNb();}
  /// @brief Same as the ActionClient closing function. Link all the concepts loaded from files and the Internet. Before closing an ontology, exploration requests are not allowed
  /// @return false if the service call fails
  bool close() {return actions.close(); }

  /// @brief If verbose is set to true, the clients will post messages about the failure to call the services and about their restoration
  void verbose(bool verbose) { ClientBase::verbose(verbose); }

  /// @brief ROS service client to query ontologenius about individuals
  IndividualClient individuals;
  /// @brief ROS service client to query ontologenius about object properties
  ObjectPropertyClient objectProperties;
  /// @brief ROS service client to query ontologenius about data properties
  DataPropertyClient dataProperties;
  /// @brief ROS service client to query ontologenius about classes
  ClassClient classes;
  /// @brief ROS service client to manage the ontology instance
  ActionClient actions;
  /// @brief ROS service client to manage reasoners plugins
  ReasonerClient reasoners;
  /// @brief ROS Publisher to insert and delete knowledge dynamically
  FeederPublisher feeder;
  /// @brief ROS service client to make SPAQRL queries
  SparqlClient sparql;
};

} // namespace onto

#endif // ONTOLOGENIUS_ONTOLOGYMANIPULATOR_H
