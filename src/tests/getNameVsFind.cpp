#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>     /* srand, rand */
#include <ctime>
#include <unordered_set>

#include <ros/ros.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

class FileReader
{
public:
  FileReader(const std::string& path)
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
    char * line = nullptr;
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

OntologyManipulator* onto_ptr;

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

void insertWords(const std::vector<std::string>& words, bool individual)
{
  ros::Rate wait(10000);
  if(individual == false)
  {
    onto_ptr->feeder.addConcept("tmp");
    for(auto& word : words)
    {
      onto_ptr->feeder.addInheritage("tmp", word);
      onto_ptr->feeder.addLanguage(word, "en", word);
      wait.sleep();
    }
  }
  else
    for(auto& word : words)
    {
      onto_ptr->feeder.addConcept(word);
      wait.sleep();
    }

}

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
    for(size_t j = 0; j < 1; j++)
      for(auto& word : words)
        nb += onto_ptr->classes.getNames(word).size();
  }
  else
  {
    for(size_t j = 0; j < 1; j++)
      for(auto& word : words)
        nb += onto_ptr->individuals.getNames(word).size();
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

  onto.verbose(true);

  onto.actions.reset();
  onto.close();
  wait.sleep();

  std::vector<size_t> nb_words = {100, 500, 1000, 5000, 10000, 50000, 100000, 450000};
  std::vector<double> finds, gettedNames;

  for(auto& nb_word : nb_words)
  {
    std::cout << "will insert" << std::endl;
    std::vector<std::string> words = readNbWords(nb_word);
    insertWords(words, false);

    std::cout << "wait" << std::endl;
    onto.feeder.waitUpdate(10000);

    finds.push_back(find(words, false) / nb_word);
    gettedNames.push_back(getName(words, false) / nb_word);

    onto.actions.reset();
    onto.close();
    wait.sleep();

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

  std::vector<std::string> words = readNbWords(450000);
  insertWords(words, false);
  std::cout << "wait" << std::endl;
  onto.feeder.waitUpdate(10000);

  std::cout << "mean words = " << getNames(words, false) << std::endl;

  return 0;
}
