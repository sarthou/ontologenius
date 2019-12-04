#ifndef ONTOLOGENIUS_ONTOLOGIESMANIPULATOR_H
#define ONTOLOGENIUS_ONTOLOGIESMANIPULATOR_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include "ontoloGenius/utility/clients/ManagerClient.h"
#include "ontoloGenius/utility/OntologyManipulator.h"

class OntologiesManipulator : public ManagerClient
{
public:
  OntologiesManipulator(ros::NodeHandle* n);
  ~OntologiesManipulator();

  void waitInit(int32_t timeout = -1);

  OntologyManipulator* operator[](const std::string& name);
  OntologyManipulator* get(const std::string& name);

  bool add(const std::string& name);
  bool copy(const std::string& dest_name, const std::string& src_name);
  bool del(const std::string& name);

  void verbose(bool verbose) { ClientBase::verbose(verbose); }

private:
  ros::NodeHandle* n_;
  std::map<std::string, OntologyManipulator*> manipulators_;

};

#endif // ONTOLOGENIUS_ONTOLOGIESMANIPULATOR_H
