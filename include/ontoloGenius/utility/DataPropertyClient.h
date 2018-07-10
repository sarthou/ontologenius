#ifndef DATAPROPERTYCLIENT_H
#define DATAPROPERTYCLIENT_H

#include "ontoloGenius/utility/ClientBase.h"

class DataPropertyClient : public ClientBase
{
public:
  DataPropertyClient(ros::NodeHandle* n) : ClientBase(n, "data_property")
  {
  }

  std::vector<std::string> getDown(const std::string& name, int depth = -1);
  std::vector<std::string> getDisjoint(const std::string& name);
  std::vector<std::string> getDomain(const std::string& name);
  std::vector<std::string> getRange(const std::string& name);

private:

};

#endif
