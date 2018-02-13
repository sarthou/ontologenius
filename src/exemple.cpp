#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/PropertyGraph.h"
#include "ontoloGenius/ontoGraphs/GraphDrawer.h"
#include "ontoloGenius/ontoGraphs/OntologyReader.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree");

{
  ClassGraph onto;
  PropertyGraph propOnto(&onto);
  OntologyReader reader(&onto, &propOnto);

  reader.readFromUri("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/attribute.owl");

  onto.close();

  GraphDrawer drawer(&onto);
  drawer.put_in_layers();
  drawer.draw("attribute.png");
}

{
  ClassGraph onto;
  PropertyGraph propOnto(&onto);
  OntologyReader reader(&onto, &propOnto);

  reader.readFromUri("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/measure.owl");

  onto.close();

  GraphDrawer drawer(&onto);
  drawer.put_in_layers();
  drawer.draw("measure.png");
}

  ROS_DEBUG("Drawing done");

  return 0;
}
