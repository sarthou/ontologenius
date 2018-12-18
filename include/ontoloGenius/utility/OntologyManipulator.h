#ifndef ONTOLOGYMANIPULATOR_H
#define ONTOLOGYMANIPULATOR_H

#include "ontoloGenius/utility/clients/ontologyClients/IndividualClient.h"
#include "ontoloGenius/utility/clients/ontologyClients/ObjectPropertyClient.h"
#include "ontoloGenius/utility/clients/ontologyClients/DataPropertyClient.h"
#include "ontoloGenius/utility/clients/ontologyClients/ClassClient.h"
#include "ontoloGenius/utility/clients/ActionClient.h"
#include "ontoloGenius/utility/clients/ArguerClient.h"
#include "ontoloGenius/utility/FeederPublisher.h"

#include "ros/ros.h"

#include <vector>
#include <string>

class OntologyManipulator
{
public:
  OntologyManipulator(ros::NodeHandle* n, const std::string& name = "");
  ~OntologyManipulator() {}

  size_t nb() {return actions.nb();}
  void resetNb() {actions.resetNb();}
  bool close() {return actions.close(); }

  IndividualClient individuals;
  ObjectPropertyClient objectProperties;
  DataPropertyClient dataProperties;
  ClassClient classes;
  ActionClient actions;
  ArguerClient arguers;
  FeederPublisher feeder;

private:
  ros::NodeHandle* n_;

};

#endif
