#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "ontologenius/OntologeniusService.h"

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

double tester(std::vector<std::string>& actions, const std::string& service)
{
  double res = 0;
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/" + service, true);
  for(size_t i = 0; i < NB_PER_SERVICE; i++)
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    for(auto& action : actions)
    {
      ontologenius::OntologeniusService srv;
      srv.request.action = action;
      srv.request.param = "this_is_a_test";

      if(!client.call(srv))
        return -1;
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "[ " << i/(NB_PER_SERVICE/100.0) << "%] " << time_span.count()/actions.size() << std::endl;
    res += time_span.count()/actions.size();
  }
  return res;
}

double classTester()
{
  std::vector<std::string> actions = {"getDown", "getUp", "getDisjoint", "getName", "find"};
  return tester(actions, "class");
}

double objectPropertyTester()
{
  std::vector<std::string> actions = {"getDown", "getUp", "getDisjoint", "getName", "find", "getInverse", "getDomain", "getRange"};
  return tester(actions, "object_property");
}

double dataPropertyTester()
{
  std::vector<std::string> actions = {"getDown", "getUp", "getDisjoint", "getName", "find", "getDomain", "getRange"};
  return tester(actions, "data_property");
}

double individualTester()
{
  std::vector<std::string> actions = {"getSame", "getDistincts", "getRelationFrom", "getRelatedFrom", "getRelationOn", "getRelatedOn", "getRelationWith", "getRelatedWith", "getUp", "getOn", "getFrom", "getWith", "getName", "find", "getType"};
  return tester(actions, "individual");
}

double reasonerTester()
{
  std::vector<std::string> actions = {"activate", "deactivate", "list", "getDescription"};
  return tester(actions, "reasoner");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_services_tester");

  ros::NodeHandle n;
  n_ = &n;

  ros::service::waitForService("ontologenius/reasoner", -1);
  close();

  double class_time = classTester();
  double objetc_time = objectPropertyTester();
  double data_time = dataPropertyTester();
  double indiv_time = individualTester();
  double reasoner_time = reasonerTester();

  double total = class_time + objetc_time + data_time + indiv_time + reasoner_time;
  total = total/5.;

  std::cout << "classes = " << class_time << std::endl;
  std::cout << "objects = " << objetc_time << std::endl;
  std::cout << "datas   = " << data_time << std::endl;
  std::cout << "indivs  = " << indiv_time << std::endl;
  std::cout << "reasoners = " << reasoner_time << std::endl;

  bool err = false;
  if(class_time < 0) {err = true; std::cout << "class service error" << std::endl; }
  if(objetc_time < 0) {err = true; std::cout << "object property service error" << std::endl; }
  if(data_time < 0) {err = true; std::cout << "data property service error" << std::endl; }
  if(indiv_time < 0) {err = true; std::cout << "individual service error" << std::endl; }
  if(reasoner_time < 0) {err = true; std::cout << "reasoner service error" << std::endl; }

  if(err == false)
  {
    std::cout << "mean = " << total << " per service"<< std::endl;
    std::cout << "mean = " << total/NB_PER_SERVICE << " per request" << std::endl;
    return 0;
  }
  else
    return -1;
}
