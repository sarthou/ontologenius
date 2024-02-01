#include <chrono>
#include <iostream>

#include <ros/ros.h>

#include "ontologenius/API/ontologenius/OntologiesManipulator.h"

using namespace std::chrono;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_dynamic_tester");

  onto::OntologiesManipulator onto;
  onto.waitInit();

  onto.add("bob");
  onto["bob"]->close();

  onto.copy("cpy", "bob");

  onto["cpy"]->feeder.waitConnected();

  onto["cpy"]->feeder.addConcept("bob");
  onto["cpy"]->feeder.addInheritage("bob", "human");
  std::string first_commit = onto["cpy"]->feeder.commit();

  onto["cpy"]->feeder.addProperty("bob", "eat", "pasta");
  onto["cpy"]->feeder.addProperty("pasta", "isIn", "bob");
  onto["cpy"]->feeder.commit("after_pasta");

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  onto["cpy"]->feeder.checkout(first_commit);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto time_span = duration_cast<std::chrono::microseconds>(t2 - t1);
  std::cout << "  " << time_span.count() << " us for checkout " << std::endl;

  onto["cpy"]->feeder.addProperty("bob", "eat", "burger");
  onto["cpy"]->feeder.addProperty("burger", "isIn", "bob");
  onto["cpy"]->feeder.commit("after_burger");

  onto["cpy"]->feeder.checkout("after_pasta");
  onto["cpy"]->feeder.addProperty("bob", "isHungry", "bool", "true");
  std::string hungry_commit = onto["cpy"]->feeder.commit();

  onto["cpy"]->feeder.checkout("after_pasta");
  onto["cpy"]->feeder.addProperty("bob", "isHungry", "bool", "false");
  onto["cpy"]->feeder.commit();

  onto["cpy"]->feeder.checkout(hungry_commit);
  std::vector<std::string> hungry_state = onto["cpy"]->individuals.getOn("bob", "isHungry");
  std::cout << "-> test hungry on cpy" << std::endl;
  for(auto& state : hungry_state)
    std::cout << "bob isHungry " << state << std::endl;

  hungry_state = onto["bob"]->individuals.getOn("bob", "isHungry");
  std::cout << "-> test hungry on bob" << std::endl;
  for(auto& state : hungry_state)
    std::cout << "bob isHungry " << state << std::endl;

  onto["cpy"]->actions.exportToXml("/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/save.xml");
  onto.del("cpy");

  ros::spinOnce();

  return 0;
}
