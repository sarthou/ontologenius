#ifndef ONTOLOGENIUS_REASONERCLIENT_H
#define ONTOLOGENIUS_REASONERCLIENT_H

#include "ontoloGenius/utility/clients/ClientBase.h"

class ReasonerClient : public ClientBase
{
public:
  ReasonerClient(ros::NodeHandle* n, const std::string& name) : ClientBase(n, (name == "") ? "reasoner" : "reasoner/" + name)
  {
  }

  std::vector<std::string> list();
  std::vector<std::string> activeList();
  bool activate(const std::string& name);
  bool deactivate(const std::string& name);
  std::string getDescription(const std::string& name);

private:

};

#endif // ONTOLOGENIUS_REASONERCLIENT_H
