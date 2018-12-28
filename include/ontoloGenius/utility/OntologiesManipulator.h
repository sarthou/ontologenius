#ifndef ONTOLOGIESMANIPULATOR_H
#define ONTOLOGIESMANIPULATOR_H

#include "ontoloGenius/utility/clients/ManagerClient.h"
#include "ontoloGenius/utility/OntologyManipulator.h"

#include "ros/ros.h"

#include <vector>
#include <string>

class OntologiesManipulator : public ManagerClient
{
public:
  OntologiesManipulator(ros::NodeHandle* n);
  ~OntologiesManipulator();

  void waitInit(int32_t timeout = -1);

  OntologyManipulator* operator[](const std::string& name);
  OntologyManipulator* get(const std::string& name);

  bool add(const std::string& name);
  bool del(const std::string& name);

private:
  ros::NodeHandle* n_;
  std::map<std::string, OntologyManipulator*> manipulators_;

};

#endif
