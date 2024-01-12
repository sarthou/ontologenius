#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>     /* srand, rand */
#include <ctime>
#include <unordered_set>

#include <ros/ros.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

using namespace std::chrono;

onto::OntologyManipulator* onto_ptr;

void insertWords(size_t nb)
{
  ros::Rate wait(10000);
  for(size_t i = 0; i < nb; i++)
  {
    onto_ptr->feeder.addProperty("individual" + std::to_string(i), "isOn", "apple");
    wait.sleep();
  }

  if(!onto_ptr->feeder.waitUpdate(10000))
    std::cout << "too long" << std::endl;
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
  const std::vector<std::string> plants = onto_ptr->individuals.getType("Plant");
  for(auto& plant : plants)
  {
    std::vector<std::string> none = onto_ptr->individuals.getFrom("isAt", plant);
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_ORO_tester");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.verbose(true);

  std::vector<size_t> nb_words = {100, 500, 1000, 5000, 10000, 50000, 100000, 450000};
  std::vector<std::vector<double> > res;
  ros::Rate wait(0.2);

  for(auto& nb_word : nb_words)
  {
    onto.actions.reset();
    onto.actions.fadd("/home/gsarthou/openrobots/share/ontologies/testsuite.owl");
    onto.close();

    wait.sleep();

    insertWords(nb_word);

    std::vector<double> tmp;
    tmp.push_back(R1());
    tmp.push_back(R2());
    tmp.push_back(R3());

    std::cout << "*********" << std::endl;
    for(auto& result : tmp)
      std::cout << result << std::endl;
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

  return 0;
}
