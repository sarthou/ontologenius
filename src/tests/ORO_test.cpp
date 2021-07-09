#include <chrono>
#include <cstdlib>     /* srand, rand */
#include <ctime>       /* time */
#include <unordered_set>
#include <thread>

#include <ros/ros.h>

#include "ontologenius/RosInterface.h"
#include "ontologenius/API/ontologenius/OntologyManipulator.h"

using namespace std::chrono;

void insertN(ontologenius::RosInterface* interface, size_t n)
{
  interface->end_feed_ = false;

  for(size_t i = 0; i < n; i++)
  {
    std::string id = std::to_string(i);
    interface->feeder_.store("[add]indiv" + id + "|isOn|apple");
  }

  std::cout << "waiting" << std::endl;

  while(interface->end_feed_ == false)
  {
    usleep(10);
  }
}

double doQuery_R1(OntologyManipulator* onto_ptr)
{
  std::cout << "R1" << std::endl;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<std::string> res = onto_ptr->individuals.getType("Animal");

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Found " << res.size() << " individuals." << std::endl;
  std::cout << "Reasoning level 1: " << " statement retrieved in " << time_span.count()*1000 << "ms" << std::endl;

  return time_span.count();
}

double doQuery_R2(OntologyManipulator* onto_ptr)
{
  std::cout << "R2" << std::endl;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<std::string> res;
  const std::vector<std::string> plants = onto_ptr->individuals.getType("Plant");
  for(auto& plant : plants)
  {
    std::vector<std::string> none = onto_ptr->individuals.getFrom("isAt", plant);
    res.insert(res.end(), none.begin(), none.end());
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Found " << res.size() << " individuals." << std::endl;
  std::cout << "Reasoning level 2: " << " statement retrieved in " << time_span.count()*1000 << "ms" << std::endl;

  return time_span.count();
}

double doQuery_R3(OntologyManipulator* onto_ptr)
{
  std::cout << "R3" << std::endl;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<std::string> res = onto_ptr->individuals.getOn("apple", "isUnder");

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Found " << res.size() << " individuals." << std::endl;
  std::cout << "Reasoning level 3: " << " statement retrieved in " << time_span.count()*1000 << "ms" << std::endl;

  return time_span.count();
}

void reset(ontologenius::RosInterface* interface, ontologenius::Ontology** onto)
{
  interface->lock();
  delete interface->onto_;
  interface->onto_ = new ontologenius::Ontology();
  interface->onto_->setDisplay(false);
  interface->reasoners_.link(interface->onto_);
  interface->feeder_.link(interface->onto_);
  interface->sparql_.link(interface->onto_);
  *onto = interface->onto_;
  interface->release();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_tester");
  ros::NodeHandle nh;

  ontologenius::Ontology* onto = nullptr;
  ontologenius::RosInterface interface(&nh);
  std::thread onto_thread;

  interface.setDisplay(false);
  interface.init("en", "none", {}, "none");
  onto = interface.getOntology();
  onto_thread = std::thread(&ontologenius::RosInterface::run, &interface);

  OntologyManipulator onto_manip;

  onto->readFromFile("/home/gsarthou/openrobots/share/ontologies/testsuite.owl");
  interface.close();

  std::vector<size_t> sizes;
  std::vector<double> r1, r2, r3;

  std::vector<size_t> nb_words = {100, 500, 1000, 5000, 10000, 50000, 100000, 450000};
  for(auto i : nb_words)
  {
    insertN(&interface, i);
    sizes.push_back(i);

    /*r1.push_back(doQuery_R1(&onto_manip));
    r2.push_back(doQuery_R2(&onto_manip));
    r3.push_back(doQuery_R3(&onto_manip));*/

    reset(&interface, &onto);
    onto->readFromFile("/home/gsarthou/openrobots/share/ontologies/testsuite.owl");
    interface.close();
  }

  for(size_t i = 0; i < sizes.size(); i++)
  {
    std::cout << sizes[i] << ", " << r1[i] << ", " << r2[i] << ", " << r3[i] << std::endl;
  }

  interface.stop();
  onto_thread.join();

  return 0;
}
