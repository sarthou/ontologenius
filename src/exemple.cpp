#include "ontoloGenius/ontoGraphs/Ontology.h"
#include "ontoloGenius/ontoGraphs/Drawers/OntologyDrawer.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree");

{
  Ontology onto;

  //onto.readFromUri("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/attribute.owl");
  onto.readFromUri("https://raw.githubusercontent.com/LAAS-HRI/semantic_route_description/master/files/route_cost.owl");
  onto.readFromUri("https://raw.githubusercontent.com/LAAS-HRI/semantic_route_description/master/files/place_description.owl");
  onto.readFromUri("https://raw.githubusercontent.com/LAAS-HRI/semantic_route_description/blob/master/files/adream_mall.owl");

  onto.close();

  OntologyDrawer drawer(&onto);
  drawer.draw("adream_mall");
}

/*{
  Ontology onto;

  onto.readFromUri("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/measure.owl");

  onto.close();

  OntologyDrawer drawer(&onto);
  drawer.draw("measure");
}*/

  ROS_DEBUG("Drawing done");

  return 0;
}
