#ifndef OBJECTPROPERTYCLIENT_H
#define OBJECTPROPERTYCLIENT_H

#include "ontoloGenius/utility/ClientBase.h"

class ObjectPropertyClient : public ClientBase
{
public:
  ObjectPropertyClient(ros::NodeHandle* n) : ClientBase(n, "object_property")
  {
  }

  std::vector<std::string> getDown(const std::string& name, int depth = -1);
  std::vector<std::string> getDisjoint(const std::string& name);
  std::vector<std::string> getDomain(const std::string& name);
  std::vector<std::string> getRange(const std::string& name);
  std::vector<std::string> getInverse(const std::string& name);

private:

};

#endif
