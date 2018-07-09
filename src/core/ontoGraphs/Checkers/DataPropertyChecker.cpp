#include "ontoloGenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"
#include <unordered_set>

size_t DataPropertyChecker::check()
{
  checkDisjoint();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void DataPropertyChecker::checkDisjoint()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    std::unordered_set<std::string> up = property_graph_->getUp(graph_[i]->value_);
    std::unordered_set<std::string> disjoint;

    std::unordered_set<std::string>::iterator it;
    for (it = up.begin(); it != up.end(); it++)
    {
      std::unordered_set<std::string> tmp = property_graph_->getDisjoint((std::string&)*it);
      disjoint.insert(tmp.begin(), tmp.end());
    }

    std::string intersection = findIntersection(up, disjoint);
    if(intersection != "")
      print_error("'" + graph_[i]->value_ + "' can't be a '" + intersection + "' because of disjonction");
  }
}
