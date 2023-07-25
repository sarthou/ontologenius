#ifndef ONTOLOGENIUS_REASONERCLIENT_H
#define ONTOLOGENIUS_REASONERCLIENT_H

#include "ontologenius/API/ontologenius/clients/ClientBase.h"

namespace onto {

/// @brief The ReasonerClient class provides an abstraction ontologenius reasoner ROS services.
/// The reasoner client is used to manage reasoner plugins for an ontology instance.
/// This makes it possible to activate or deactivate plugins.
/// This class is based on ClientBase and so ensure a persistent connection with the service based on.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class ReasonerClient : public ClientBase
{
public:
  /// @brief Constructs a reasoner client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param n is an initialized ROS node handle.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  ReasonerClient(ros::NodeHandle* n, const std::string& name) : ClientBase(n, (name == "") ? "reasoner" : "reasoner/" + name)
  {
  }

  /// @brief Gets the name of the plugins available.
  /// @return the list as the available plugins in the form of a verctor of string.
  std::vector<std::string> list();
  /// @brief Gets the name of the activated plugins.
  /// @return the list as the activated plugins in the form of a verctor of string.
  std::vector<std::string> activeList();
  /// @brief Activate a reasoner.
  /// @param name is the name of the reasoner to be activated.
  /// @return false if the service call fails.
  bool activate(const std::string& name);
  /// @brief Deactivate a reasoner.
  /// @param name is the name of the reasoner to be deactivated.
  /// @return false if the service call fails.
  bool deactivate(const std::string& name);
  /// @brief Gets the description of a given reasoner.
  /// @param name is the name of the reasoner you want to get the description
  /// @return The description in the form of a string
  std::string getDescription(const std::string& name);

private:

};

} // namespace onto

#endif // ONTOLOGENIUS_REASONERCLIENT_H
