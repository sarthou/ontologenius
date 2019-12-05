#ifndef ONTOLOGENIUS_ONTOLOGYCLIENT_H
#define ONTOLOGENIUS_ONTOLOGYCLIENT_H

#include "ontologenius/API/ontologenius/clients/ClientBase.h"

class OntologyClient : public ClientBase
{
public:
  OntologyClient(ros::NodeHandle* n, std::string name) : ClientBase(n, name)
  {
  }

  std::vector<std::string> getUp(const std::string& name, int depth = -1, const std::string& selector = "");
  bool isA(const std::string& name, const std::string& base_class);
  std::string getName(const std::string& name, bool take_id = true);
  std::vector<std::string> getNames(const std::string& name);
  std::vector<std::string> getEveryNames(const std::string& name);
  std::vector<std::string> find(const std::string& name);
  std::vector<std::string> findSub(const std::string& name);
  std::vector<std::string> findRegex(const std::string& regex);
  std::vector<std::string> findFuzzy(const std::string& name, double threshold = 0.5);
  bool exist(const std::string& name);
};

#endif // ONTOLOGENIUS_ONTOLOGYCLIENT_H
