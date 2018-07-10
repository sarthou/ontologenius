#include "ontoloGenius/core/ontoGraphs/Ontology.h"
#include "ontoloGenius/core/arguer/Arguers.h"

#include "ros/ros.h"

#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <unordered_set>

using namespace std::chrono;

std::string set2string(std::unordered_set<std::string> word_set)
{
  std::string result = "";
  for(const std::string& it : word_set)
    result += it + " ";
  return result;
}

std::vector<std::string> generate_sequence(ClassGraph& onto)
{
  srand (time(NULL));

  std::vector<ClassBranch_t*> base = onto.get();
  size_t max_index = base.size();

  std::string _1_10_1 = base[rand() % max_index]->value_;
  std::string _1_10_2 = base[rand() % max_index]->value_;

  std::string _1_20_1 = base[rand() % max_index]->value_;
  std::string _1_20_2 = base[rand() % max_index]->value_;

  std::string _1_25_1 = base[rand() % max_index]->value_;
  std::string _1_25_2 = base[rand() % max_index]->value_;
  std::string _1_25_3 = base[rand() % max_index]->value_;

  std::string _1_50_1 = base[rand() % max_index]->value_;
  std::string _1_50_2 = base[rand() % max_index]->value_;
  std::string _1_50_3 = base[rand() % max_index]->value_;
  std::string _1_50_4 = base[rand() % max_index]->value_;
  std::string _1_50_5 = base[rand() % max_index]->value_;

  std::vector<std::string> vect20;
  vect20.resize(10);
  vect20[0] = _1_10_1;
  vect20[1] = _1_10_2;
  vect20.insert(vect20.end(), vect20.begin(), vect20.end());//20
  vect20[2] = _1_20_1;
  vect20[12] = _1_20_2;
  std::vector<std::string> vect100 = vect20;
  vect100.insert(vect100.end(), vect20.begin(), vect20.end());//40
  vect100.insert(vect100.end(), vect20.begin(), vect20.end());//60
  vect100.insert(vect100.end(), vect20.begin(), vect20.end());//80
  vect100.insert(vect100.end(), vect20.begin(), vect20.end());//100
  for(int i = 3; i < 100; i += 25)
    vect100[i] = _1_25_1;
  for(int i = 13; i < 100; i += 25)
    vect100[i] = _1_25_2;
  for(int i = 23; i < 100; i += 25)
    vect100[i] = _1_25_3;

  for(int i = 33; i < 100; i += 50)
    vect100[i] = _1_50_1;
  for(int i = 43; i < 100; i += 50)
    vect100[i] = _1_50_2;
  for(int i = 4; i < 100; i += 50)
    vect100[i] = _1_50_3;
  for(int i = 14; i < 100; i += 50)
    vect100[i] = _1_50_4;

  vect100[24] = base[rand() % max_index]->value_;
  vect100[34] = base[rand() % max_index]->value_;

  std::vector<std::string> vect10000 = vect100;
  for(int i = 0; i < 100; i++)
    vect10000.insert(vect10000.end(), vect100.begin(), vect100.end());

  for(size_t i = 0; i < vect10000.size(); i++)
    if(vect10000[i] == "")
      vect10000[i] = base[rand() % max_index]->value_;

  return vect10000;
}

double testOne(std::vector<std::string>& seq, Ontology& onto)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(size_t i = 0; i < seq.size(); i++)
    std::unordered_set<std::string> out = onto.class_graph_.getUp(seq[i]);

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "  " << time_span.count() << " for " << seq.size() << " getUp " << std::endl;
  return time_span.count();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_tester");

  Ontology onto;
  Arguers arguers(&onto);
  arguers.load();
  arguers.list();

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  onto.readFromFile("/home/osboxes/LAAS/catkin_ws/src/ontologenius/files/attribute.owl");
  onto.readFromFile("/home/osboxes/LAAS/catkin_ws/src/ontologenius/files/positionProperty.owl");
  onto.readFromFile("/home/osboxes/LAAS/catkin_ws/src/ontologenius/files/testIndividuals.owl");
  onto.readFromFile("/home/osboxes/LAAS/catkin_ws/src/ontologenius/files/property.owl");
  onto.readFromFile("/home/osboxes/LAAS/catkin_ws/src/ontologenius/files/measure.owl");

  onto.close();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "It took me " << time_span.count() << " seconds to read" << std::endl;
  double read_time = time_span.count();

  arguers.runPostArguers();

  int epoch = 10000;

  std::vector<std::string> seq = generate_sequence(onto.class_graph_);
  double total = 0;
  for(int i = 0; i < epoch; i++)
  {
    std::cout << "[ " << i/(epoch/100.0) << "%]";
    total += testOne(seq, onto);
  }

  std::cout << "mean = " << total/((float)epoch) << " per sequence of " << seq.size() << std::endl;
  std::cout << "mean = " << total/((float)epoch)/seq.size() << " per request" << std::endl;
  std::cout << "It took me " << read_time << " seconds to read" << std::endl;

  ROS_DEBUG("Drawing done");

  return 0;
}
