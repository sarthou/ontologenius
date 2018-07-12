#include "ontoloGenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"
#include <unordered_set>

size_t DataPropertyChecker::check()
{
  graph_size = graph_.size();
  checkDisjoint();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void DataPropertyChecker::checkDisjoint()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<std::string> up = property_graph_->getUp(graph_[i]->value());
    std::unordered_set<std::string> disjoint;

    for (std::string it : up)
    {
      std::unordered_set<std::string> tmp = property_graph_->getDisjoint(it);
      disjoint.insert(tmp.begin(), tmp.end());
    }

    std::string intersection = findIntersection(up, disjoint);
    if(intersection != "")
      print_error("'" + graph_[i]->value() + "' can't be a '" + intersection + "' because of disjonction");
  }
}
