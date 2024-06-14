#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib> /* srand, rand */
#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <unordered_set>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"
#include "ontologenius/interface/RosInterface.h"

class FileReader
{
public:
  explicit FileReader(const std::string& path)
  {
    len = cpt = 0;
    file_ = nullptr;
    init(path, "r");
  }

  ~FileReader()
  {
    if(file_ != nullptr)
      fclose(file_);
  }

  std::string readLine()
  {
    char* line = nullptr;
    if(getline(&line, &len, file_) != -1)
    {
      cpt++;
      return std::string(line).substr(0, std::string(line).size() - 1);
    }
    else
      return "";
  }

  size_t getNbLine() { return cpt; }

private:
  size_t len;
  size_t cpt;

  void init(const std::string& file_name, const std::string& option)
  {
    file_ = fopen(file_name.c_str(), option.c_str());
    if(file_ == nullptr)
      std::cout << "Fail to open file " << file_name << " with option '" << option << "'" << std::endl;
  }

  void reset(const std::string& file_name)
  {
    file_ = fopen(file_name.c_str(), "w");
    if(file_ == nullptr)
      std::cout << "Fail to reset file " << file_name << std::endl;
    else
      fclose(file_);
    file_ = nullptr;
  }

  FILE* file_;
};

using namespace std::chrono;

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
    if(word.empty())
      oef = true;
    else
    {
      if(cpt % (size_t(factor)) == 0)
        res.push_back(word);
    }
    cpt++;
  } while((oef == false) && (res.size() != nb));
  std::cout << res.size() << std::endl;

  return res;
}

void insertWords(ontologenius::RosInterface* interface, const std::vector<std::string>& words, bool individual)
{
  interface->end_feed_ = false;

  if(individual == false)
  {
    interface->feeder_.store("[add]tmp|");
    for(auto& word : words)
    {
      interface->feeder_.store("[add]" + word + "|+|tmp");
      interface->feeder_.store("[add]" + word + "|@en|" + word);
    }
  }
  else
  {
    for(auto& word : words)
      interface->feeder_.store("[add]" + word + "|");
  }

  std::cout << "waiting" << std::endl;

  while(interface->end_feed_ == false)
  {
    usleep(10);
  }
}

onto::OntologyManipulator* onto_ptr;

double find(const std::vector<std::string>& words, bool individual)
{
  size_t syn = 0;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  if(individual == false)
  {
    for(size_t j = 0; j < 3; j++)
      for(auto& word : words)
      {
        std::vector<std::string> none = onto_ptr->classes.find(word);
        syn += none.size();
      }
  }
  else
  {
    for(size_t j = 0; j < 3; j++)
      for(auto& word : words)
      {
        std::vector<std::string> none = onto_ptr->individuals.find(word);
        syn += none.size();
      }
  }
  (void)syn;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  return time_span.count() * 1000;
}

double getName(const std::vector<std::string>& words, bool individual)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  if(individual == false)
  {
    for(size_t j = 0; j < 1; j++)
      for(auto& word : words)
        onto_ptr->classes.getName(word);
  }
  else
  {
    for(size_t j = 0; j < 1; j++)
      for(auto& word : words)
        onto_ptr->individuals.getName(word);
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  return time_span.count() * 1000 / 1;
}

double getNames(const std::vector<std::string>& words, bool individual)
{
  double nb = 0;

  if(individual == false)
  {
    for(auto& word : words)
      nb += onto_ptr->classes.getNames(word).size();
  }
  else
  {
    for(auto& word : words)
      nb += onto_ptr->individuals.getNames(word).size();
  }

  return nb / words.size();
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
  ros::init(argc, argv, "ontologenius_responseTime_tester");

  ros::NodeHandle nh;
  ontologenius::Ontology* onto = nullptr;
  ontologenius::RosInterface interface;
  std::thread onto_thread;

  interface.setDisplay(false);
  interface.init("en", "none", {}, "none");
  onto = interface.getOntology();
  onto_thread = std::thread(&ontologenius::RosInterface::run, &interface);
  interface.close();

  onto::OntologyManipulator onto_manip;
  onto_ptr = &onto_manip;

  std::vector<size_t> nb_words = {100, 500, 1000, 5000, 10000, 50000, 100000, 450000};
  std::vector<double> finds, gettedNames;

  for(auto& nb_word : nb_words)
  {
    std::cout << "will insert" << std::endl;
    std::vector<std::string> words = readNbWords(nb_word);
    insertWords(&interface, words, true);

    finds.push_back(find(words, true) / nb_word);
    gettedNames.push_back(getName(words, true) / nb_word);

    reset(&interface, &onto);
    interface.close();

    std::cout << "*********" << std::endl;
    for(size_t j = 0; j < finds.size(); j++)
      std::cout << nb_words[j] << ";";
    std::cout << std::endl;
    for(auto& find_time : finds)
      std::cout << find_time << ";";
    std::cout << std::endl;
    for(auto& get_name_time : gettedNames)
      std::cout << get_name_time << ";";
    std::cout << std::endl;
    std::cout << "*********" << std::endl;
  }

  {
    std::vector<std::string> words = readNbWords(450000);
    insertWords(&interface, words, true);

    std::cout << "mean words = " << getNames(words, true) << std::endl;
  }

  {
    std::vector<std::string> words = readNbWords(466508);
    size_t s = 0, min = 10000, max = 0;
    for(auto& w : words)
    {
      s += w.size();
      if(w.size() > max)
        max = w.size();
      if(w.size() < min)
        min = w.size();
    }
    std::cout << "nb word " << words.size() << std::endl;
    std::cout << "mean = " << s / 466508.0 << std::endl;
    std::cout << "min/max " << min << " : " << max << std::endl;
  }

  interface.stop();
  onto_thread.join();

  return 0;
}
