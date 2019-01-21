#ifndef VALIDITYCHECKER_H
#define VALIDITYCHECKER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontoloGenius/core/utility/color.h"

#include <string>
#include <iostream>
#include <unordered_set>

template <typename B>
class ValidityChecker
{
  static_assert(std::is_base_of<ValuedNode,B>::value, "B must be derived from ValuedNode");
public:
  ValidityChecker(Graph<B>* graph)
  {
    graph_vect_ = graph->get();
    graph_size = graph_vect_.size();
    nb_error_ = 0;
    is_analysed = false;
  }
  virtual ~ValidityChecker() {}

  virtual size_t check() = 0;

protected:
  std::vector<B*> graph_vect_;
  bool is_analysed;
  size_t graph_size;

  void print_error(std::string err)
  {
    std::cout << COLOR_RED << err << COLOR_OFF << std::endl;
    nb_error_++;
  }

  virtual void printStatus() = 0;

  void printStatus(std::string type, std::string types, size_t nb)
  {
    if(is_analysed)
    {
      if(nb_error_)
        std::cout << COLOR_RED << nb_error_ << " errors " << COLOR_OFF << std::endl;

      if(nb_error_)
        std::cout << COLOR_RED << "Failure of " << type << " analysis" << COLOR_OFF;
      else
        std::cout << COLOR_GREEN << "Succeed " << type << " analysis" << COLOR_OFF;

      std::cout << " : " << nb << " " << types << " analyzed" << std::endl;
    }
    else
    {
      std::cout << COLOR_RED << type << " not analysed " << COLOR_OFF;
      std::cout << " : " << nb << " " << types << " to analyze" << std::endl;
    }
  }

  std::string findIntersection(std::unordered_set<std::string>& base, std::unordered_set<std::string>& comp)
  {
    std::string res = "";
    std::unordered_set<std::string>::iterator it;
    for (it = comp.begin(); it != comp.end(); it++)
    {
      std::unordered_set<std::string>::iterator find = base.find(*it);
      if(find != base.end())
        res = *it;
    }
    return res;
  }

  std::unordered_set<std::string> checkLoops()
  {
    std::unordered_set<std::string> errors;

    return errors;
  }



  size_t getErrors() {return nb_error_; }

private:
  size_t nb_error_;
};

#endif
