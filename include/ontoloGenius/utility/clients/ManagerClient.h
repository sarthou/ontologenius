#ifndef ONTOLOGENIUS_MANAGERCLIENT_H
#define ONTOLOGENIUS_MANAGERCLIENT_H

#include "ontoloGenius/utility/clients/ClientBase.h"

class ManagerClient : public ClientBase
{
public:
  ManagerClient(ros::NodeHandle* n) : ClientBase(n, "manage")
  {
  }

  std::vector<std::string> list();
  bool add(const std::string& name);
  bool copy(const std::string& dest_name, const std::string& src_name);
  bool del(const std::string& name);
  std::vector<std::string> getDifference(const std::string& onto1, const std::string& onto2, const std::string& concept);

private:

};

#endif // ONTOLOGENIUS_MANAGERCLIENT_H
