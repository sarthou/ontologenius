#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologiesManipulator.h"

#include <chrono>
#include <iostream>

using namespace std::chrono;

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
  for(size_t i = 0; i < 12; i++)
    onto["cpy"]->feeder.addProperty("pasta", "isIn", "bob" + std::to_string(i));
  std::string commit3 = onto["cpy"]->feeder.commit();

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  onto["cpy"]->feeder.checkout(commit1);

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto time_span = duration_cast<std::chrono::milliseconds>(t2 - t1);

  std::cout << "  " << time_span.count() << " ms for checkout " << std::endl;

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
