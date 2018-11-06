#ifndef ACTIONCLIENT_H
#define ACTIONCLIENT_H

#include "ontoloGenius/utility/ClientBase.h"

class ActionClient : public ClientBase
{
public:
  ActionClient(ros::NodeHandle* n) : ClientBase(n, "actions")
  {
  }

  bool close();
  bool setLang(const std::string& lang);
  bool add(const std::string& uri);
  bool fadd(const std::string& file);
  bool reset();

private:

};

#endif
