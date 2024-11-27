#include <chrono>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <ros/ros.h>
#include <thread>
#include <unordered_set>

#define ONTO_TEST

#include "ontologenius/interface/RosInterface.h"

using namespace std::chrono;

#include <fstream>
#include <ios>
#include <iostream>
#include <string>
#include <unistd.h>
using namespace std;

constexpr std::string test_file = "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/ontologenius/files/Compare/knowrob.owl"

  void
  mem_usage(double& vm_usage, double& resident_set)
{
  vm_usage = 0.0;
  resident_set = 0.0;
  ifstream stat_stream("/proc/self/stat", ios_base::in); // get info from proc
  // create some variables to get info
  string pid, comm, state, ppid, pgrp, session, tty_nr;
  string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  string utime, stime, cutime, cstime, priority, nice;
  string O, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
  stat_stream.close();
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;
}

void insertN(ontologenius::RosInterface* interface, size_t n)
{
  interface->end_feed_ = false;

  for(size_t i = 0; i < n; i++)
  {
    std::string id = std::to_string(i);
    interface->feeder_.store("[add]VP" + id + "|+|VisualPerception");
    interface->feeder_.store("[add]cup" + id + "|+|Cup");
    interface->feeder_.store("[add]VP" + id + "|actOn|cup" + id);
    interface->feeder_.store("[add]VP" + id + "|occursAt|mat#[[1,0,0,2.56], [0,1,0,1.32],[0,0,1,0.38],[0,0,0,1]]");
    interface->feeder_.store("[add]VP" + id + "|startTime|time#0.125");
  }

  std::cout << "waiting" << std::endl;

  while(interface->end_feed_ == false)
  {
    usleep(10);
  }
}

bool doQuery(ontologenius::Ontology* onto, size_t n)
{
  int rnd = rand() % n;
  ontologenius::ClassBranch* cup = onto->class_graph_.findBranchSafe("Cup");
  ontologenius::IndividualBranch* cup_i = cup->individual_childs_[rnd].elem;
  auto VP_i = onto->individual_graph_.getOn(cup_i->value(), "Vp_has_obj");
  if(VP_i.empty() == false)
  {
    auto pose = onto->individual_graph_.getOn(*(VP_i.begin()), "occursAt");
    return (pose.empty() == false);
  }
  std::cout << "no VP" << std::endl;
  return false;
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

  ontologenius::Ontology* onto = nullptr;
  ontologenius::RosInterface interface;
  std::thread onto_thread;

  interface.setDisplay(false);
  interface.init("en", "none", {}, "none");
  onto = interface.getOntology();
  onto_thread = std::thread(&ontologenius::RosInterface::run, &interface);
  onto->readFromFile(test_file);
  interface.close();

  std::vector<size_t> sizes;
  std::vector<double> times;
  std::vector<double> time_per_obj;
  std::vector<double> mem;
  std::vector<double> query;

  for(size_t i = 100; i < 1000000; i = i * 10)
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    insertN(&interface, i);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);
    sizes.push_back(i);
    times.push_back(time_span1.count());
    time_per_obj.push_back(time_span1.count() / i);
    std::cout << "It took me " << time_span1.count() << " seconds to create " << i << std::endl;
    std::cout << "--> " << time_span1.count() / i << " seconds per obj" << std::endl;

    high_resolution_clock::time_point t3 = high_resolution_clock::now();
    bool query_ok = doQuery(onto, i);
    high_resolution_clock::time_point t4 = high_resolution_clock::now();
    duration<double> time_span2 = duration_cast<duration<double>>(t4 - t3);
    std::cout << "It took me " << time_span2.count() << " seconds to query on " << i << std::endl;
    query.push_back(time_span2.count());
    if(query_ok == false)
      std::cout << "pb with query" << std::endl;

    double vm, rss;
    mem_usage(vm, rss);
    cout << "Virtual Memory: " << vm << " Resident set size: " << rss << endl;
    mem.push_back(rss);

    reset(&interface, &onto);
    onto->readFromFile(test_file);
    interface.close();
  }

  for(size_t i = 200000; i <= 1000000; i += 200000) // 1000000
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    insertN(&interface, i);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);
    sizes.push_back(i);
    times.push_back(time_span1.count());
    time_per_obj.push_back(time_span1.count() / i);
    std::cout << "It took me " << time_span1.count() << " seconds to create " << i << std::endl;
    std::cout << "--> " << time_span1.count() / i << " seconds per obj" << std::endl;

    high_resolution_clock::time_point t3 = high_resolution_clock::now();
    bool query_ok = doQuery(onto, i);
    high_resolution_clock::time_point t4 = high_resolution_clock::now();
    duration<double> time_span2 = duration_cast<duration<double>>(t4 - t3);
    std::cout << "It took me " << time_span2.count() << " seconds to query on " << i << std::endl;
    query.push_back(time_span2.count());
    if(query_ok == false)
      std::cout << "pb with query" << std::endl;

    double vm, rss;
    mem_usage(vm, rss);
    cout << "Virtual Memory: " << vm << " Resident set size: " << rss << endl;
    mem.push_back(rss);

    reset(&interface, &onto);
    onto->readFromFile(test_file);
    interface.close();
  }

  for(size_t i = 0; i < sizes.size(); i++)
  {
    std::cout << sizes[i] << ", " << times[i] << ", " << time_per_obj[i] << ", " << query[i] << ", " << mem[i] << std::endl;
  }

  interface.stop();
  onto_thread.join();

  return 0;
}
