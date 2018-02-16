#include "ontoloGenius/ontoGraphs/Ontology.h"

#include "ros/ros.h"

#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <set>

using namespace std::chrono;

std::string set2string(std::set<std::string> word_set)
{
  std::string result = "";
  for(std::set<std::string>::iterator it = word_set.begin(); it != word_set.end(); ++it)
    result += *it + " ";
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

  for(int i = 0; i < vect10000.size(); i++)
    if(vect10000[i] == "")
      vect10000[i] = base[rand() % max_index]->value_;

  return vect10000;
}

double testOne(ClassGraph& onto)
{
  std::vector<std::string> seq = generate_sequence(onto);

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(int i = 0; i < seq.size(); i++)
    std::set<std::string> out = onto.getUp(seq[i]);

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "It took me " << time_span.count() << std::endl;
  return time_span.count();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tester");

  Ontology onto;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  onto.readFromFile("/home/gsarthou/Desktop/test.owl");
  onto.readFromFile("/home/gsarthou/Desktop/attribute.owl");
  onto.readFromFile("/home/gsarthou/Desktop/positionProperty.owl");
  onto.readFromFile("/home/gsarthou/Desktop/testIndividuals.owl");

  onto.close();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "It took me " << time_span.count() << " seconds to read" << std::endl;

  std::string tmp = "cube1";
  std::cout << "=" << set2string(onto.individuals_.getUp(tmp)) << std::endl;

  /*double total = 0;
  for(int i = 0; i < 10000; i++)
  {
    std::cout << "[ " << i/100. << "%]";
    total += testOne(onto.classes_);
  }

  std::cout << "mean = " << total/10000.0 << std::endl;*/

  ROS_DEBUG("Drawing done");

  return 0;
}
