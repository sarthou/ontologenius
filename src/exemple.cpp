#include "ontoloGenius/treeObject.h"
#include "ontoloGenius/treeProperty.h"
#include "ontoloGenius/tree_drawer.h"
#include "ontoloGenius/ontology_reader.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree");

{
  treeObject onto;
  treeProperty propOnto;
  Ontology_reader reader(&onto, &propOnto);

  reader.read("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/attribute.owl");

  onto.close();

  tree_drawer drawer(&onto);
  drawer.put_in_layers();
  drawer.draw("attribute.png");
}

{
  treeObject onto;
  treeProperty propOnto;
  Ontology_reader reader(&onto, &propOnto);

  reader.read("https://raw.githubusercontent.com/sarthou/toaster/master/tools/Ontology/measure.owl");

  onto.close();

  tree_drawer drawer(&onto);
  drawer.put_in_layers();
  drawer.draw("measure.png");
}

  ROS_DEBUG("Drawing done");

  return 0;
}
