#include "ontoloGenius/ontoGraphs/Ontology.h"
#include "ontoloGenius/ontoGraphs/GraphDrawer.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree");

{
  Ontology onto;

  onto.readFromUri("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/attribute.owl");

  onto.close();

  GraphDrawer drawer(&onto.classes_);
  drawer.put_in_layers();
  drawer.draw("attribute.png");
}

{
  Ontology onto;

  onto.readFromUri("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/measure.owl");

  onto.close();

  GraphDrawer drawer(&onto.classes_);
  drawer.put_in_layers();
  drawer.draw("measure.png");
}

  ROS_DEBUG("Drawing done");

  return 0;
}
