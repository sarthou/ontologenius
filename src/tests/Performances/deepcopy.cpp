#include <iostream>
#include <chrono>
#include <cstdlib>     /* srand, rand */
#include <ctime>       /* time */

#include <ros/ros.h>

#include "ontologenius/API/ontologenius/OntologiesManipulator.h"

using namespace std::chrono;

void insertN(onto::OntologyManipulator* onto, size_t n)
{
  onto->feeder.waitConnected();
  for(size_t i = 0; i < n; i++)
  {
    std::string id = std::to_string(i);
    onto->feeder.addInheritage("VP" + id, "VisualPerception");
    onto->feeder.addInheritage("cup" + id, "Cup");
    onto->feeder.addProperty("VP" + id, "actOn", "cup" + id);
    onto->feeder.addProperty("VP" + id, "occursAt", "mat", "[[1,0,0,2.56], [0,1,0,1.32],[0,0,1,0.38],[0,0,0,1]]");
    onto->feeder.addProperty("VP" + id, "startTime", "time", "0.125");
    usleep(10);
  }

  std::cout << "waiting" << std::endl;

  onto->feeder.waitUpdate();
}

double deepCopy(onto::OntologiesManipulator* onto, size_t n)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  onto->copy("new", "base" + std::to_string(n));

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  onto->del("new");

  std::cout << time_span.count() << std::endl;
  return time_span.count();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_multi_tester");

  onto::OntologiesManipulator onto;

  std::vector<size_t> sizes;
  std::vector<double> res;

  for(size_t i = 100; i <= 1000000; i= i*10)
  {
    std::string base_name = "base" + std::to_string(i);
    onto.add(base_name);
    onto[base_name]->actions.fadd("/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/ontologenius/files/test.owl");
    onto[base_name]->close();
    insertN(onto[base_name], i);

    sizes.push_back(i);
    res.push_back(deepCopy(&onto, i));
    onto.del(base_name);
  }

  for(size_t i = 0; i < sizes.size(); i++)
  {
    std::cout << sizes[i] << ", " << res[i] << std::endl;
  }

  return 0;
}
