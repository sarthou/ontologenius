#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologiesManipulator.h"

#include <iostream>

OntologiesManipulator* onto_ptr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_dynamic_tester");

  ros::NodeHandle n;
  OntologiesManipulator onto(&n);
  onto_ptr = &onto;
  onto.waitInit();

  onto.add("robot");
  onto["robot"]->close();

  ros::Rate wait(1);
  wait.sleep();

  onto.copy("cpy", "robot");

  onto["cpy"]->feeder.waitConnected();

  onto["cpy"]->feeder.addConcept("bob");
  onto["cpy"]->feeder.addInheritage("bob", "human");
  std::string commit1 = onto["cpy"]->feeder.commit();

  onto["cpy"]->feeder.addProperty("bob", "eat", "pasta");
  std::string commit2 = onto["cpy"]->feeder.commit();

  onto["cpy"]->feeder.addProperty("pasta", "isIn", "bob");
  std::string commit3 = onto["cpy"]->feeder.commit();

  onto["cpy"]->feeder.checkout(commit1);

  onto["cpy"]->feeder.addProperty("bob", "eat", "burger");
  std::string commit4 = onto["cpy"]->feeder.commit();

  onto["cpy"]->feeder.addProperty("burger", "isIn", "bob");
  std::string commit5 = onto["cpy"]->feeder.commit();

  onto["cpy"]->feeder.checkout(commit3);

  onto["cpy"]->feeder.addProperty("bob", "isHungry", "bool", "false");
  std::string commit6 = onto["cpy"]->feeder.commit();

  onto["cpy"]->actions.exportToXml("/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/save.xml");


  return 0;
}
