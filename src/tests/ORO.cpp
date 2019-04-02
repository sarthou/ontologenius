#include "ontoloGenius/utility/OntologyManipulator.h"

#include "ros/ros.h"

#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <unordered_set>
#include <stdio.h>
#include <math.h>
#include <atomic>

using namespace std::chrono;

OntologyManipulator* onto_ptr;
std::atomic<bool> end_;

void insertWords(size_t nb)
{
  ros::Rate wait(10000);
  for(size_t i = 0; i < nb; i++)
  {
    onto_ptr->feeder.addProperty("individual" + std::to_string(i), "isOn", "apple");
    wait.sleep();
  }
}

double R1()
{
  std::cout << "R1" << std::endl;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<std::string> res = onto_ptr->individuals.getType("Animal");

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Found " << res.size() << " individuals." << std::endl;
  std::cout << "Reasoning level 1: " << 1 << " statements retrieved in " << time_span.count()*1000 << "ms (" << 1 / time_span.count() << " stmt/sec)." << std::endl;

  return time_span.count()*1000;
}

double R2()
{
  std::cout << "R2" << std::endl;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<std::string> res;
  std::vector<std::string> plant = onto_ptr->individuals.getType("Plant");
  for(size_t i = 0; i < plant.size(); i++)
  {
    std::vector<std::string> none = onto_ptr->individuals.getFrom("isAt", plant[i]);
    res.insert(res.end(), none.begin(), none.end());
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Found " << res.size() << " individuals." << std::endl;
  std::cout << "Reasoning level 2: " << 1 << " statements retrieved in " << time_span.count()*1000 << "ms (" << 1 / time_span.count() << " stmt/sec)." << std::endl;

  return time_span.count()*1000;
}

double R3()
{
  std::cout << "R3" << std::endl;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<std::string> res = onto_ptr->individuals.getOn("apple", "isUnder");

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Found " << res.size() << " individuals." << std::endl;
  std::cout << "Reasoning level 3: " << 1 << " statements retrieved in " << time_span.count()*1000 << "ms (" << 1 / time_span.count() << " stmt/sec)." << std::endl;

  return time_span.count()*1000;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  end_ = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_ORO_tester");

  ros::NodeHandle n;
  OntologyManipulator onto(&n);
  onto_ptr = &onto;

  ros::Subscriber sub = n.subscribe("ontologenius/end", 1000, chatterCallback);

  onto.verbose(true);

  std::vector<size_t> nb_words = {100, 500, 1000, 5000, 10000, 50000, 100000, 450000};
  std::vector<std::vector<double> > res;
  ros::Rate wait(0.2);
  ros::Rate fast(100);

  for(size_t i = 0; i < nb_words.size(); i++)
  {
    onto.actions.reset();
    onto.actions.fadd("/home/gsarthou/openrobots/share/ontologies/testsuite.owl");
    onto.close();

    wait.sleep();
    end_ = false;

    insertWords(nb_words[i]);

    while(end_ == false)
    {
      ros::spinOnce();
      fast.sleep();
    }

    std::vector<double> tmp;
    tmp.push_back(R1());
    tmp.push_back(R2());
    tmp.push_back(R3());

    std::cout << "*********" << std::endl;
    for(size_t j = 0; j < tmp.size(); j++)
      std::cout << tmp[j] << std::endl;
    res.push_back(tmp);
  }

  std::cout << "***************" << std::endl;
  std::cout << "***************" << std::endl;
  for(size_t i = 0; i < res[0].size(); i++)
  {
    for(size_t j = 0; j < res.size(); j++)
      std::cout << res[j][i] << ";";
    std::cout << std::endl;
  }

  ROS_DEBUG("test done");

  return 0;
}
