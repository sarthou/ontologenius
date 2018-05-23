#ifndef VALIDITYCHECKER_H
#define VALIDITYCHECKER_H

#include "ontoloGenius/core/ontoGraphs/Graphs/OntoGraph.h"

#include <string>
#include <iostream>
#include <set>

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN   "\x1B[1;92m"
#endif

template <typename B>
class ValidityChecker
{
  static_assert(std::is_base_of<ValuedNode,B>::value, "B must be derived from ValuedNode");
public:
  ValidityChecker(Graph<B>* graph) {graph_ = graph->get(); nb_error_ = 0; is_analysed = false;}
  virtual ~ValidityChecker() {}

  virtual size_t check() = 0;

protected:
  std::vector<B*> graph_;
  bool is_analysed;

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

  std::string findIntersection(std::set<std::string>& base, std::set<std::string>& comp)
  {
    std::string res = "";
    std::set<std::string>::iterator it;
    for (it = comp.begin(); it != comp.end(); it++)
    {
      std::set<std::string>::iterator find = base.find(*it);
      if(find != base.end())
        res = *it;
    }
    return res;
  }

  size_t getErrors() {return nb_error_; }

private:
  size_t nb_error_;
};

#endif
