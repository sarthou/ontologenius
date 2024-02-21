#ifndef ONTOLOGENIUS_VALIDITYCHECKER_H
#define ONTOLOGENIUS_VALIDITYCHECKER_H

#include <string>
#include <iostream>
#include <unordered_set>

#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

template <typename B>
class ValidityChecker
{
  static_assert(std::is_base_of<ValuedNode,B>::value, "B must be derived from ValuedNode");
public:
  ValidityChecker(Graph<B>* graph)
  {
    graph_vect_ = graph->get();
    nb_error_ = 0;
    nb_warn_ = 0;
    is_analysed = false;
  }
  virtual ~ValidityChecker() {}

  virtual size_t check() = 0;

protected:
  std::vector<B*> graph_vect_;
  bool is_analysed;

  void print_error(std::string err)
  {
    Display::error(err);
    nb_error_++;
  }

  void print_warning(std::string warn)
  {
    Display::warning(warn);
    nb_warn_++;
  }

  virtual void printStatus() = 0;

  void printStatus(const std::string& type, const std::string& types, size_t nb)
  {
    if(is_analysed)
    {
      if(nb_error_)
        Display::error(std::to_string(nb_error_) + " errors ");

      if(nb_warn_)
        Display::warning(std::to_string(nb_warn_) + " warnings ");

      if(nb_error_)
        Display::error("Failure of " + type + " analysis", false);
      else
        Display::success("Succeed " + type + " analysis", false);

      std::cout << " : " << nb << " " << types << " analyzed" << std::endl;
    }
    else
    {
      Display::error(type + " not analysis", false);
      std::cout << " : " << nb << " " << types << " to analyze" << std::endl;
    }
  }

  size_t getErrors() {return nb_error_; }
  size_t getWarnings() {return nb_warn_; }

private:
  size_t nb_error_;
  size_t nb_warn_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_VALIDITYCHECKER_H
