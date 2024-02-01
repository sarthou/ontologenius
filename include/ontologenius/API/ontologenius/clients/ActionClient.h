#ifndef ONTOLOGENIUS_ACTIONCLIENT_H
#define ONTOLOGENIUS_ACTIONCLIENT_H

#include "ontologenius/API/ontologenius/clients/ClientBase.h"

namespace onto {

/// @brief The ActionClient class provides an abstraction ontologenius action ROS services.
/// This class is based on ClientBase and so ensure a persistent connection with the service based on.
/// The persistent connection ensures a minimal response time.
/// A reconnection logic is implemented in the event that the persistent connection fails. 
class ActionClient : public ClientBase
{
public:
  /// @brief Constructs an action client.
  /// Can be used in a multi-ontology mode by specifying the name of the ontology name.
  /// @param name is the instance to be connected to. For classic use, name should be defined as "".
  explicit ActionClient(const std::string& name) : ClientBase((name == "") ? "actions" : "actions/" + name)
  {
  }

  /// @brief Link all the concepts loaded from files and the Internet. Before closing an ontology, exploration requests are not allowed.
  /// @return Returns false ontology closure fails or if the service call fails.
  bool close();
  /// @brief Saves the current ontology in an absolute path.
  /// @param path is the path where the ontology will be saved. It must be of the form: my/path/to/ontology.owl
  /// @return Returns false if the service call fails.
  bool save(const std::string& path);
  /// @brief Exports the current modification tree in an absolute path.
  /// This function has no effect on non copied ontologies.
  /// @param path is the path where the modification tree will be saved. It must be of the form: my/path/to/file.xml
  /// @return Returns false if the service call fails.
  bool exportToXml(const std::string& path);
  /// @brief Sets the language of work.
  /// @param lang is the language inedntifier (e.g. en, fr, ...)
  /// @return Returns false if the service call fails.
  bool setLang(const std::string& lang);
  /// @brief Gets the wotking language.
  /// @return Returns the working language.
  std::string getLang();
  /// @brief Loads an ontology file (.owl) from the internet. The Close function should be called after all the desired files have been loaded.
  /// @param uri is the ontology file web adress.
  /// @return Returns false if the service call fails.
  bool add(const std::string& uri);
  /// @brief Loads an ontology file (.owl) stored on your local computer. The Close function should be called after all the desired files have been loaded.
  /// @param file is the local path to the ontology file.
  /// @return Returns false if the service call fails.
  bool fadd(const std::string& file);
  /// @brief Unloads all the knowledge previously loaded or learned and reload the default files.
  /// @return Returns false if the service call fails.
  bool reset();
  /// @brief Unloads all the knowledge previously loaded or learned.
  /// @return Returns false if the service call fails.
  bool clear();

private:

};

} // namespace onto

#endif // ONTOLOGENIUS_ACTIONCLIENT_H
