#ifndef ONTOLOGENIUS_INDIVIDUALCLIENT_H
#define ONTOLOGENIUS_INDIVIDUALCLIENT_H

#include "ontoloGenius/utility/clients/ontologyClients/OntologyClient.h"

class IndividualClient : public OntologyClient
{
public:
  IndividualClient(ros::NodeHandle* n, const std::string& name) : OntologyClient(n, (name == "")? "individual" : "individual/" + name)
  {
  }

  std::vector<std::string> getOn(const std::string& name, const std::string& property, const std::string& selector = "");
  std::vector<std::string> getFrom(const std::string& property, const std::string& name, const std::string& selector = "");
  std::vector<std::string> getWith(const std::string& indiv_from, const std::string& indiv_to, const std::string& selector = "", int depth = -1);

  std::vector<std::string> getRelatedFrom(const std::string& property);
  std::vector<std::string> getRelatedOn(const std::string& property);
  std::vector<std::string> getRelatedWith(const std::string& name);

  std::vector<std::string> getRelationFrom(const std::string& name, int depth = -1);
  std::vector<std::string> getRelationOn(const std::string& name, int depth = -1);
  std::vector<std::string> getRelationWith(const std::string& name);

  std::vector<std::string> getType(const std::string& name);
  std::vector<std::string> getSame(const std::string& name);
  std::vector<std::string> getDistincts(const std::string& name);

private:

};

#endif // ONTOLOGENIUS_INDIVIDUALCLIENT_H
