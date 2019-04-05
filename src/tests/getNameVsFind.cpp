#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <unordered_set>
#include <stdio.h>
#include <math.h>
#include <atomic>

#include <ros/ros.h>

#include "ontoloGenius/utility/OntologyManipulator.h"

class FileReader
{
public:
  FileReader(std::string path)
  {
    len = cpt = 0;
    file_ = NULL;
    init(path, "r");
  }
  ~FileReader()
  {
    if(file_ != NULL)
      fclose(file_);
  }

  std::string readLine()
  {
    char * line = NULL;
    if(getline(&line, &len, file_) != -1)
    {
        cpt++;
        return  std::string(line).substr(0,std::string(line).size()-1);
    }
    else
      return "";
  }

  size_t getNbLine() {return cpt; }
private:
  size_t len;
  size_t cpt;

  void init(std::string file_name, std::string option)
  {
    file_ = fopen(file_name.c_str(), option.c_str());
    if(file_ == NULL)
      std::cout << "Fail to open file " << file_name << " with option '" << option << "'" << std::endl;
  }

  void reset(std::string file_name)
  {
    file_ = fopen(file_name.c_str(), "w");
    if(file_ == NULL)
      std::cout << "Fail to reset file " << file_name << std::endl;
    else
      fclose(file_);
    file_ = NULL;
  }

  FILE* file_;
};

using namespace std::chrono;

OntologyManipulator* onto_ptr;
std::atomic<bool> end_;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  end_ = true;
  std::cout << "end" << std::endl;
}

std::vector<std::string> readNbWords(size_t nb)
{
  std::vector<std::string> res;
  float factor = 466509 / nb;
  size_t cpt = 0;

  FileReader reader("/home/gsarthou/Desktop/words.txt");
  bool oef = false;

  do
  {
    std::string word = reader.readLine();
    if(word == "")
      oef = true;
    else
    {
      if(cpt%(size_t(factor)) == 0)
        res.push_back(word);
    }
    cpt++;
  }
  while((oef == false) && (res.size() != nb));
  std::cout << res.size() << std::endl;

  return res;
}

void insertWords(std::vector<std::string>& words, bool individual)
{
  ros::Rate wait(10000);
  if(individual == false)
  {
    onto_ptr->feeder.addConcept("tmp");
    for(size_t i = 0; i < words.size(); i++)
    {
      onto_ptr->feeder.addInheritage("tmp", words[i]);
      onto_ptr->feeder.addLanguage(words[i], "en", words[i]);
      wait.sleep();
    }
  }
  else
    for(size_t i = 0; i < words.size(); i++)
    {
      onto_ptr->feeder.addConcept(words[i]);
      wait.sleep();
    }

}

double find(std::vector<std::string>& words, bool individual)
{
  size_t syn = 0;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  if(individual == false)
  {
    for(size_t j = 0; j < 3; j++)
      for(size_t i = 0; i < words.size(); i++)
      {
        std::vector<std::string> none = onto_ptr->classes.find(words[i]);
        syn += none.size();
      }
  }
  else
  {
    for(size_t j = 0; j < 3; j++)
      for(size_t i = 0; i < words.size(); i++)
      {
        std::vector<std::string> none = onto_ptr->individuals.find(words[i]);
        syn += none.size();
      }
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  return time_span.count() * 1000 /1;
}

double getName(std::vector<std::string>& words, bool individual)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  if(individual == false)
  {
    for(size_t j = 0; j < 1; j++)
      for(size_t i = 0; i < words.size(); i++)
        onto_ptr->classes.getName(words[i]);
  }
  else
  {
    for(size_t j = 0; j < 1; j++)
      for(size_t i = 0; i < words.size(); i++)
        onto_ptr->individuals.getName(words[i]);
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  return time_span.count() * 1000 / 1;
}

double getNames(std::vector<std::string>& words, bool individual)
{
  double nb = 0;

  if(individual == false)
  {
    for(size_t j = 0; j < 1; j++)
      for(size_t i = 0; i < words.size(); i++)
        nb += onto_ptr->classes.getNames(words[i]).size();
  }
  else
  {
    for(size_t j = 0; j < 1; j++)
      for(size_t i = 0; i < words.size(); i++)
        nb += onto_ptr->individuals.getNames(words[i]).size();
  }

  return nb/words.size();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_responseTime_tester");

  ros::NodeHandle n;
  OntologyManipulator onto(&n);
  onto_ptr = &onto;

  ros::Rate wait(1);
  ros::Rate fast(100);

  ros::Subscriber sub = n.subscribe("ontologenius/end", 1000, chatterCallback);

  onto.verbose(true);

  onto.actions.reset();
  onto.close();
  wait.sleep();

  std::vector<size_t> nb_words = {100, 500, 1000, 5000, 10000, 50000, 100000, 450000};
  std::vector<double> finds, gettedNames;

  for(size_t i = 0; i < nb_words.size(); i++)
  {
    end_ = false;
    std::cout << "will insert" << std::endl;
    std::vector<std::string> words = readNbWords(nb_words[i]);
    insertWords(words, false);

    std::cout << "wait" << std::endl;
    while(end_ == false)
    {
      ros::spinOnce();
      fast.sleep();
    }

    finds.push_back(find(words, false) / nb_words[i]);
    gettedNames.push_back(getName(words, false) / nb_words[i]);

    onto.actions.reset();
    onto.close();
    wait.sleep();

    std::cout << "*********" << std::endl;
    for(size_t j = 0; j < finds.size(); j++)
      std::cout << nb_words[j] << ";";
    std::cout << std::endl;
    for(size_t j = 0; j < finds.size(); j++)
      std::cout << finds[j] << ";";
    std::cout << std::endl;
    for(size_t j = 0; j < finds.size(); j++)
      std::cout << gettedNames[j] << ";";
    std::cout << std::endl;
    std::cout << "*********" << std::endl;
  }

  std::vector<std::string> words = readNbWords(450000);
  insertWords(words, false);

  std::cout << "wait" << std::endl;
  while(end_ == false)
  {
    ros::spinOnce();
    fast.sleep();
  }

  std::cout << "mean words = " << getNames(words, false) << std::endl;


  ROS_DEBUG("test done");

  return 0;
}
