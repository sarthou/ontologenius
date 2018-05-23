#include "ontoloGenius/core/ontoGraphs/Ontology.h"
#include "ontoloGenius/graphical/Drawers/OntologyDrawer.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologeniusExemple");

{
  Ontology onto;

  onto.readFromUri("https://raw.githubusercontent.com/sarthou/ontologenius/master/files/attribute.owl");
  onto.readFromUri("https://raw.githubusercontent.com/sarthou/ontologenius/master/files/measure.owl");
  onto.readFromUri("https://raw.githubusercontent.com/sarthou/ontologenius/master/files/positionProperty.owl");
  onto.readFromUri("https://raw.githubusercontent.com/sarthou/ontologenius/master/files/property.owl");
  onto.readFromUri("https://raw.githubusercontent.com/sarthou/ontologenius/master/files/testIndividuals.owl");

  onto.close();

  OntologyDrawer drawer(&onto);
  drawer.draw("ontologenius_exemple");
}

  ROS_DEBUG("Drawing done");

  return 0;
}
