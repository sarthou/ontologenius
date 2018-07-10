#ifndef ONTOLOGYMANIPULATOR_H
#define ONTOLOGYMANIPULATOR_H

#include "ontoloGenius/utility/IndividualClient.h"

#include "ros/ros.h"

#include <vector>
#include <string>

class OntologyManipulator
{
public:
  OntologyManipulator(ros::NodeHandle* n);
  ~OntologyManipulator() {}


  size_t nb() {individuals.nb();}
  void reset() {individuals.reset();}

  bool close();

  IndividualClient individuals;

private:
  ros::NodeHandle* n_;

};

#endif
