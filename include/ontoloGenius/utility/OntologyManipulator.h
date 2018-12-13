#ifndef ONTOLOGYMANIPULATOR_H
#define ONTOLOGYMANIPULATOR_H

#include "ontoloGenius/utility/IndividualClient.h"
#include "ontoloGenius/utility/ObjectPropertyClient.h"
#include "ontoloGenius/utility/DataPropertyClient.h"
#include "ontoloGenius/utility/ClassClient.h"
#include "ontoloGenius/utility/ActionClient.h"
#include "ontoloGenius/utility/ArguerClient.h"
#include "ontoloGenius/utility/FeederPublisher.h"

#include "ros/ros.h"

#include <vector>
#include <string>

class OntologyManipulator
{
public:
  OntologyManipulator(ros::NodeHandle* n);
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
