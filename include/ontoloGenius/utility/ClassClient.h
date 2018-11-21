#ifndef CLASSCLIENT_H
#define CLASSCLIENT_H

#include "ontoloGenius/utility/OntologyClient.h"

class ClassClient : public OntologyClient
{
public:
  ClassClient(ros::NodeHandle* n) : OntologyClient(n, "class")
  {
  }

  std::vector<std::string> getDown(const std::string& name, int depth = -1);
  std::vector<std::string> getDisjoint(const std::string& name);

private:

};

#endif
