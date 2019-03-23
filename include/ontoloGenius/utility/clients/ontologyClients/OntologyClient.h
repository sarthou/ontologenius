#ifndef ONTOLOGYCLIENT_H
#define ONTOLOGYCLIENT_H

#include "ontoloGenius/utility/clients/ClientBase.h"

class OntologyClient : public ClientBase
{
public:
  OntologyClient(ros::NodeHandle* n, std::string name) : ClientBase(n, name)
  {
  }

  std::vector<std::string> getUp(const std::string& name, int depth = -1, const std::string& selector = "");
  bool isA(std::string& name, const std::string& base_class);
  std::string getName(const std::string& name);
  std::vector<std::string> getNames(const std::string& name);
  std::vector<std::string> find(const std::string& name);
};

#endif
