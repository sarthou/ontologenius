#ifndef CLASSCLIENT_H
#define CLASSCLIENT_H

#include "ontoloGenius/utility/ClientBase.h"

class ClassClient : public ClientBase
{
public:
  ClassClient(ros::NodeHandle* n) : ClientBase(n, "class")
  {
  }

  std::vector<std::string> getDown(const std::string& name, int depth = -1);
  std::vector<std::string> getDisjoint(const std::string& name);

private:

};

#endif
