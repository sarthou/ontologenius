#ifndef ONTOLOGENIUS_OBJECTPROPERTYCLIENT_H
#define ONTOLOGENIUS_OBJECTPROPERTYCLIENT_H

#include "ontoloGenius/utility/clients/ontologyClients/OntologyClient.h"

class ObjectPropertyClient : public OntologyClient
{
public:
  ObjectPropertyClient(ros::NodeHandle* n, const std::string& name) : OntologyClient(n, (name == "") ? "object_property" : "object_property/" + name)
  {
  }

  std::vector<std::string> getDown(const std::string& name, int depth = -1);
  std::vector<std::string> getDisjoint(const std::string& name);
  std::vector<std::string> getDomain(const std::string& name);
  std::vector<std::string> getRange(const std::string& name);
  std::vector<std::string> getInverse(const std::string& name);

private:

};

#endif // ONTOLOGENIUS_OBJECTPROPERTYCLIENT_H
