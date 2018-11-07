#ifndef ARGUERCLIENT_H
#define ARGUERCLIENT_H

#include "ontoloGenius/utility/ClientBase.h"

class ArguerClient : public ClientBase
{
public:
  ArguerClient(ros::NodeHandle* n) : ClientBase(n, "arguer")
  {
  }

  std::vector<std::string> list();
  bool activate(const std::string& name);
  bool deactivate(const std::string& name);
  std::string getDescription(const std::string& name);

private:

};

#endif
