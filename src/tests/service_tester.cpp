#include "ontologenius/OntologeniusService.h"

#include "ros/ros.h"
#include <iostream>
#include <chrono>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <string>

using namespace std::chrono;

#define NB_PER_SERVICE 10000

ros::NodeHandle* n_;

bool close()
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/actions");
  ontologenius::OntologeniusService srv;
  srv.request.action = "close";

  if(!client.call(srv))
    return false;
  else
    return true;
};

double classTester()
{
  double res = 0;
  std::vector<std::string> actions = {"getDown", "getUp", "getDisjoint", "getName", "find"};
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/class");
  for(size_t i = 0; i < NB_PER_SERVICE; i++)
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    for(size_t j = 0; j < actions.size(); j++)
    {
      ontologenius::OntologeniusService srv;
      srv.request.action = actions[j];
      srv.request.param = "this_is_a_test";

      if(!client.call(srv))
        return -1;
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "[ " << i/(NB_PER_SERVICE/100.0) << "%] " << time_span.count()/actions.size() << std::endl;
    res += time_span.count()/actions.size();
  }
}

double objectPropertyTester()
{
  double res = 0;
  std::vector<std::string> actions = {"getDown", "getUp", "getDisjoint", "getName", "find", "getInverse", "getDomain", "getRange"};
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/object_property");
  for(size_t i = 0; i < NB_PER_SERVICE; i++)
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    for(size_t j = 0; j < actions.size(); j++)
    {
      ontologenius::OntologeniusService srv;
      srv.request.action = actions[j];
      srv.request.param = "this_is_a_test";

      if(!client.call(srv))
        return -1;
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "[ " << i/(NB_PER_SERVICE/100.0) << "%] " << time_span.count()/actions.size() << std::endl;
    res += time_span.count()/actions.size();
  }
}

double dataPropertyTester()
{
  double res = 0;
  std::vector<std::string> actions = {"getDown", "getUp", "getDisjoint", "getName", "find", "getDomain", "getRange"};
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/data_property");
  for(size_t i = 0; i < NB_PER_SERVICE; i++)
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    for(size_t j = 0; j < actions.size(); j++)
    {
      ontologenius::OntologeniusService srv;
      srv.request.action = actions[j];
      srv.request.param = "this_is_a_test";

      if(!client.call(srv))
        return -1;
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "[ " << i/(NB_PER_SERVICE/100.0) << "%] " << time_span.count()/actions.size() << std::endl;
    res += time_span.count()/actions.size();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_services_tester");

  ros::NodeHandle n;
  n_ = &n;

  ros::service::waitForService("ontoloGenius/arguer", -1);
  close();

  double class_time = classTester();
  double objetc_time = objectPropertyTester();
  double data_time = dataPropertyTester();

  double total = class_time + objetc_time + data_time;
  total = total/3.;

  std::cout << "classes = " << class_time << std::endl;
  std::cout << "objects = " << objetc_time << std::endl;
  std::cout << "datas   = " << data_time << std::endl;
  std::cout << "mean = " << total << " per service"<< std::endl;
  std::cout << "mean = " << total/NB_PER_SERVICE << " per request" << std::endl;

  ROS_DEBUG("Service test done");

  return 0;
}
