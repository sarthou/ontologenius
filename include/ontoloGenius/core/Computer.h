#include <vector>
#include <string>
#include <unordered_set>
#include <iostream>

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"


struct finder_t
{
  std::vector<std::unordered_set<std::string>> words;
  std::vector<std::vector<bool>> find;
};

class Computer
{
public:
  Computer() {}
  ~Computer() {}

  bool compute(std::string equation, ClassGraph& onto);

private:
  std::vector<std::vector<std::string>> L;
  std::vector<std::vector<bool>> notL;
  std::vector<std::vector<std::string>> R;
  std::vector<std::vector<bool>> notR;

  bool split(const std::string &txt, std::vector<std::string> &strs, char ch);
  bool split(const std::string &txt, std::vector<std::string> &strs, std::string delim);
};
