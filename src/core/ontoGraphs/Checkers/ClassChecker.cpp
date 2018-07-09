#include "ontoloGenius/core/ontoGraphs/Checkers/ClassChecker.h"
#include <unordered_set>

size_t ClassChecker::check()
{
  checkDisjoint();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void ClassChecker::checkDisjoint()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    std::unordered_set<std::string> up = class_graph_->getUp(graph_[i]->value_);
    std::unordered_set<std::string> disjoint;

    for(std::string it : up)
    {
      std::unordered_set<std::string> tmp = class_graph_->getDisjoint(it);
      disjoint.insert(tmp.begin(), tmp.end());
    }

    std::string intersection = findIntersection(up, disjoint);
    if(intersection != "")
      print_error("'" + graph_[i]->value_ + "' can't be a '" + intersection + "' because of disjonction");
  }
}
