#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <ros/ros.h>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/BranchContainer/BranchContainerDyn.h"
#include "ontologenius/core/ontoGraphs/BranchContainer/BranchContainerMap.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

class FileReader
{
public:
  explicit FileReader(const std::string& path)
  {
    len_ = cpt_ = 0;
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
    if(getline(&line, &len_, file_) != -1)
    {
      cpt_++;
      return {line};
    }
    else
      return "";
  }

  size_t getNbLine() const { return cpt_; }

private:
  size_t len_;
  size_t cpt_;

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

std::vector<ontologenius::ValuedNode*> full_words;

void readFullWords()
{
  FileReader reader("/home/gsarthou/Desktop/words.txt");
  bool oef = false;
  do
  {
    std::string res = reader.readLine();
    if(res.empty())
      oef = true;
    else
    {
      auto* tmp = new ontologenius::ValuedNode(res);
      full_words.push_back(tmp);
    }
  } while(oef == false);
  std::cout << reader.getNbLine() << std::endl;
}

double findAll(ontologenius::BranchContainerBase<ontologenius::ValuedNode>* container, const std::vector<ontologenius::ValuedNode*>& words)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(auto* word : words)
  {
    ontologenius::ValuedNode* none = container->find(word->value());
    (void)none;
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  return time_span.count() / words.size();
}

double findAllNTime(ontologenius::BranchContainerBase<ontologenius::ValuedNode>* container, const std::vector<ontologenius::ValuedNode*>& words, size_t n)
{
  double sum = 0;
  for(size_t i = 0; i < n; i++)
  {
    sum += findAll(container, words);
  }

  return sum / n;
}

std::vector<ontologenius::ValuedNode*> getPartOfWords(float percent)
{
  std::vector<ontologenius::ValuedNode*> tmp;
  for(size_t i = 0; i < full_words.size() * percent / 100; i++)
  {
    size_t my_index = rand() % full_words.size();
    tmp.push_back(full_words[my_index]);
  }

  return tmp;
}

std::vector<ontologenius::ValuedNode*> creatTestVector(float percent, std::vector<ontologenius::ValuedNode*> words)
{
  std::vector<ontologenius::ValuedNode*> tmp;
  for(size_t i = 0; i < words.size() * percent / 100; i++)
  {
    size_t my_index = rand() % words.size();
    tmp.push_back(words[my_index]);
  }

  if(tmp.size() < full_words.size() / 2)
  {
    std::vector<ontologenius::ValuedNode*> res;
    while(res.size() < full_words.size())
    {
      res.insert(res.end(), tmp.begin(), tmp.end());
    }
    return res;
  }
  else
    return tmp;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_container_tester");

  readFullWords();

  size_t nb = 100;

  for(size_t i = 1; i <= 5; i++)
  {
    std::cout << "########" << std::endl
              << "#  " << i << "  #" << std::endl
              << "########" << std::endl
              << std::endl;
    ontologenius::BranchContainerDyn<ontologenius::ValuedNode> container_dyn;
    ontologenius::BranchContainerMap<ontologenius::ValuedNode> container_map;

    std::vector<ontologenius::ValuedNode*> part_of_words = getPartOfWords(i);
    container_dyn.load(part_of_words);
    container_map.load(part_of_words);

    for(float j = 0.1; j <= 2; j = j + 0.1)
    {
      std::vector<ontologenius::ValuedNode*> vect = creatTestVector(j, part_of_words);
      double time_span_dyn = findAllNTime(&container_dyn, vect, nb);
      double time_span_map = findAllNTime(&container_map, vect, nb);

      std::cout << i << " " << j << " " << time_span_dyn << " " << time_span_map << " " << vect.size() * nb << " " << time_span_dyn * vect.size() * nb << " " << time_span_map * vect.size() * nb << std::endl;
    }

    size_t size = full_words.size() * i / 100;
    std::cout << log2(size) << " : " << size << std::endl;
  }

  for(auto& full_word : full_words)
    delete full_word;

  ROS_DEBUG("test done");

  return 0;
}
