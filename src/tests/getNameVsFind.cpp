#include "ontoloGenius/utility/OntologyManipulator.h"

#include "ros/ros.h"

#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <unordered_set>
#include <stdio.h>
#include <math.h>

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
  ros::Rate wait(100);
  if(individual == false)
  {
    onto_ptr->feeder.addConcept("tmp");
    for(size_t i = 0; i < words.size(); i++)
    {
      onto_ptr->feeder.addInheritage("tmp", words[i]);
      //wait.sleep();
    }
  }
  else
    for(size_t i = 0; i < words.size(); i++)
    {
      onto_ptr->feeder.addConcept(words[i]);
      //wait.sleep();
    }

}

double find(std::vector<std::string>& words, bool individual)
{
  size_t syn = 0;

  if(individual == false)
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    for(size_t j = 0; j < 3; j++)
      for(size_t i = 0; i < words.size(); i++)
      {
        std::vector<std::string> none = onto_ptr->classes.getUp(words[i]);
        syn += none.size();
      }

    std::cout << syn << " SYN" << std::endl;
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    return time_span.count() /3;// (double)(words.size() * 3);
  }
  else
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    for(size_t j = 0; j < 3; j++)
      for(size_t i = 0; i < words.size(); i++)
      {
        std::vector<std::string> none = onto_ptr->individuals.find(words[i]);
        syn += none.size();
      }

    std::cout << syn << " SYN" << std::endl;
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    return time_span.count() /3;// (double)(words.size() * 3);
  }
}

double getName(std::vector<std::string>& words, bool individual)
{
  if(individual == false)
  {
    std::vector<std::string> verify;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    for(size_t j = 0; j < 3; j++)
    {
      verify.clear();
      for(size_t i = 0; i < words.size(); i++)
        verify.push_back(onto_ptr->classes.getName(words[i]));
    }

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

    size_t err = 0;
    for(size_t i = 0; i < words.size(); i++)
      if(words[i] != verify[i])
      {
        err++;
        std::cout << words[i] << " : " << verify[i] << std::endl;
      }
    std::cout << err << " errors" << std::endl;

    return time_span.count() / 3;// (double)(words.size() * 3);
  }
  else
  {
    std::vector<std::string> verify;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    for(size_t j = 0; j < 3; j++)
    {
      verify.clear();
      for(size_t i = 0; i < words.size(); i++)
        verify.push_back(onto_ptr->individuals.getName(words[i]));
    }

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

    size_t err = 0;
    std::cout << words.size() << " : " << verify.size() << std::endl;
    for(size_t i = 0; i < words.size(); i++)
      if(words[i] != verify[i])
        err++;
    std::cout << err << " errors" << std::endl;

    return time_span.count() / 3;//(double)(words.size() * 3);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_responseTime_tester");

  ros::NodeHandle n;
  OntologyManipulator onto(&n);
  onto_ptr = &onto;

  onto.verbose(true);

  onto.actions.reset();
  onto.close();

  std::vector<size_t> nb_words = {100, 500, 1000, 5000, 10000, 50000, 100000, 450000};
  std::vector<double> finds, getNames;
  ros::Rate wait(0.2);

  for(size_t i = 0; i < nb_words.size(); i++)
  {
    wait.sleep();

    std::vector<std::string> words = readNbWords(nb_words[i]);
    insertWords(words, false);

    wait.sleep();

    finds.push_back(find(words, false));
    getNames.push_back(getName(words, false));

    onto.actions.reset();
    onto.close();
  }

  for(size_t j = 0; j < nb_words.size(); j++)
    std::cout << nb_words[j] << ";";
  std::cout << std::endl;
  for(size_t j = 0; j < nb_words.size(); j++)
    std::cout << finds[j] << ";";
  std::cout << std::endl;
  for(size_t j = 0; j < nb_words.size(); j++)
    std::cout << getNames[j] << ";";
  std::cout << std::endl;

  finds.clear();
  getNames.clear();
  /*for(size_t i = 0; i < nb_words.size(); i++)
  {
    wait.sleep();

    std::vector<std::string> words = readNbWords(nb_words[i]);
    insertWords(words, true);

    wait.sleep();

    finds.push_back(find(words, true));
    getNames.push_back(getName(words, true));

    onto.actions.reset();
    onto.close();

    for(size_t j = 0; j < finds.size(); j++)
      std::cout << nb_words[j] << ";";
    std::cout << std::endl;
    for(size_t j = 0; j < finds.size(); j++)
      std::cout << finds[j] << ";";
    std::cout << std::endl;
    for(size_t j = 0; j < finds.size(); j++)
      std::cout << getNames[j] << ";";
    std::cout << std::endl;
  }*/

  ROS_DEBUG("test done");

  return 0;
}
