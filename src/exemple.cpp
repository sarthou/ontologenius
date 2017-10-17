#include "ontoloGenius/TreeObject.h"
#include "ontoloGenius/TreeProperty.h"
#include "ontoloGenius/TreeDrawer.h"
#include "ontoloGenius/OntologyReader.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree");

{
  TreeObject onto;
  TreeProperty propOnto(&onto);
  OntologyReader reader(&onto, &propOnto);

  reader.read("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/attribute.owl");

  onto.close();

  TreeDrawer drawer(&onto);
  drawer.put_in_layers();
  drawer.draw("attribute.png");
}

{
  TreeObject onto;
  TreeProperty propOnto(&onto);
  OntologyReader reader(&onto, &propOnto);

  reader.read("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/measure.owl");

  onto.close();

  TreeDrawer drawer(&onto);
  drawer.put_in_layers();
  drawer.draw("measure.png");
}

  ROS_DEBUG("Drawing done");

  return 0;
}
