#ifndef ONTOLOGENIUS_ONTOLOGYMANIPULATOR_H
#define ONTOLOGENIUS_ONTOLOGYMANIPULATOR_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include "ontologenius/API/ontologenius/clients/ontologyClients/IndividualClient.h"
#include "ontologenius/API/ontologenius/clients/ontologyClients/ObjectPropertyClient.h"
#include "ontologenius/API/ontologenius/clients/ontologyClients/DataPropertyClient.h"
#include "ontologenius/API/ontologenius/clients/ontologyClients/ClassClient.h"
#include "ontologenius/API/ontologenius/clients/ActionClient.h"
#include "ontologenius/API/ontologenius/clients/ReasonerClient.h"
#include "ontologenius/API/ontologenius/FeederPublisher.h"

class OntologyManipulator
{
public:
  OntologyManipulator(ros::NodeHandle* n, const std::string& name = "");
  ~OntologyManipulator() {}

  size_t nb() {return actions.nb();}
  void resetNb() {actions.resetNb();}
  bool close() {return actions.close(); }

  void verbose(bool verbose) { ClientBase::verbose(verbose); }

  IndividualClient individuals;
  ObjectPropertyClient objectProperties;
  DataPropertyClient dataProperties;
  ClassClient classes;
  ActionClient actions;
  ReasonerClient reasoners;
  FeederPublisher feeder;

private:
  ros::NodeHandle* n_;

};

#endif // ONTOLOGENIUS_ONTOLOGYMANIPULATOR_H
