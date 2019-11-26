#ifndef ONTOLOGENIUS_ACTIONCLIENT_H
#define ONTOLOGENIUS_ACTIONCLIENT_H

#include "ontoloGenius/utility/clients/ClientBase.h"

class ActionClient : public ClientBase
{
public:
  ActionClient(ros::NodeHandle* n, const std::string& name) : ClientBase(n, (name == "") ? "actions" : "actions/" + name)
  {
  }

  bool close();
  bool save(const std::string& path);
  bool setLang(const std::string& lang);
  std::string getLang();
  bool add(const std::string& uri);
  bool fadd(const std::string& file);
  bool reset();

private:

};

#endif // ONTOLOGENIUS_ACTIONCLIENT_H
