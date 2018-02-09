#include "ontoloGenius/ontoGraphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/PropertyGraph.h"
#include "ontoloGenius/ontoGraphs/OntologyReader.h"
#include "ros/ros.h"

#include <chrono>

using namespace std::chrono;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tester");

  ClassGraph onto;
  PropertyGraph propOnto(&onto);
  OntologyReader reader(&onto, &propOnto);

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  reader.readFromFile("/home/gsarthou/Desktop/test.owl");
  onto.close();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "It took me " << time_span.count() << " seconds to read";

  ROS_DEBUG("Drawing done");

  return 0;
}
