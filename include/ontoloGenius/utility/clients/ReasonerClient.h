#ifndef ARGUERCLIENT_H
#define ARGUERCLIENT_H

#include "ontoloGenius/utility/clients/ClientBase.h"

class ArguerClient : public ClientBase
{
public:
  ArguerClient(ros::NodeHandle* n, const std::string& name) : ClientBase(n, (name == "") ? "arguer" : "arguer/" + name)
  {
  }

  std::vector<std::string> list();
  bool activate(const std::string& name);
  bool deactivate(const std::string& name);
  std::string getDescription(const std::string& name);

private:

};

#endif
