#ifndef ONTOLOGENIUS_COMPUTER_H
#define ONTOLOGENIUS_COMPUTER_H

#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"

namespace ontologenius {

  struct finder_t
  {
    std::vector<std::unordered_set<std::string>> words;
    std::vector<std::vector<bool>> find;
  };

  class Computer
  {
  public:
    Computer() = default;
    ~Computer() = default;

    bool compute(std::string equation, ClassGraph& onto);

  private:
    std::vector<std::vector<std::string>> L;
    std::vector<std::vector<bool>> notL;
    std::vector<std::vector<std::string>> R;
    std::vector<std::vector<bool>> notR;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_COMPUTER_H
