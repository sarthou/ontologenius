#ifndef ONTOLOGENIUS_ONTOLOGYMANIPULATORINDEX_H
#define ONTOLOGENIUS_ONTOLOGYMANIPULATORINDEX_H

#include <vector>
#include <string>

#include "ontologenius/compat/ros.h"

#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/IndividualIndexClient.h"
#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/ObjectPropertyIndexClient.h"
#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/DataPropertyIndexClient.h"
#include "ontologenius/API/ontologenius/clientsIndex/ontologyClients/ClassIndexClient.h"
#include "ontologenius/API/ontologenius/clientsIndex/SparqlIndexClient.h"
#include "ontologenius/API/ontologenius/clients/ActionClient.h"
#include "ontologenius/API/ontologenius/clients/ReasonerClient.h"
#include "ontologenius/API/ontologenius/FeederPublisher.h"
#include "ontologenius/API/ontologenius/ConversionClient.h"

namespace onto {

/// @brief The OntologyManipulatorIndex class is just an object to access all API ROS abstraction classes so that you can query and manage ontologenius.
class OntologyManipulatorIndex
{
private:
  std::string name_;
public:
  OntologyManipulatorIndex(const std::string& name = "");
  /// @brief OntologyManipulatorIndex copy constructor
  /// @param other a reference to the object to copy
  OntologyManipulatorIndex(const OntologyManipulatorIndex& other);
  OntologyManipulatorIndex(OntologyManipulatorIndex&& other);
  /// @brief OntologyManipulatorIndex desconstructor
  ~OntologyManipulatorIndex() {}

  /// @brief Gives the total number of service calls from all ROS clients instances since the last reset
  size_t nb() {return actions.nb() + individuals.nb();}
  /// @brief Reset the call counter for all instances of ROS clients
  void resetNb() {actions.resetNb(); individuals.resetNb();}
  /// @brief Same as the ActionClient closing function. Link all the concepts loaded from files and the Internet. Before closing an ontology, exploration requests are not allowed
  /// @return false if the service call fails
  bool close() {return actions.close(); }

  /// @brief If verbose is set to true, the clients will post messages about the failure to call the services and about their restoration
  void verbose(bool verbose)
  {
    ClientBase::verbose(verbose);
    ClientBaseIndex::verbose(verbose);
  }

  /// @brief ROS service client to query ontologenius about individuals
  IndividualIndexClient individuals;
  /// @brief ROS service client to query ontologenius about object properties
  ObjectPropertyIndexClient objectProperties;
  /// @brief ROS service client to query ontologenius about data properties
  DataPropertyIndexClient dataProperties;
  /// @brief ROS service client to query ontologenius about classes
  ClassIndexClient classes;
  /// @brief ROS service client to manage the ontology instance
  ActionClient actions;
  /// @brief ROS service client to manage reasoners plugins
  ReasonerClient reasoners;
  /// @brief ROS Publisher to insert and delete knowledge dynamically
  FeederPublisher feeder;
  /// @brief ROS service client to make SPAQRL queries
  SparqlIndexClient sparql;
  /// @brief ROS service client to make convertions between indexes and string identifiers
  ConversionClient conversion;
};

} // namespace onto

#endif // ONTOLOGENIUS_ONTOLOGYMANIPULATORINDEX_H
