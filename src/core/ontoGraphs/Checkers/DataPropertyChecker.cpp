#include "ontoloGenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"
#include <unordered_set>

namespace ontologenius {

size_t DataPropertyChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);
  graph_size = graph_vect_.size();

  checkDisjoint();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void DataPropertyChecker::checkDisjoint()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<std::string> up = property_graph_->getUp(graph_vect_[i]->value());
    std::unordered_set<std::string> disjoint;

    for (std::string it : up)
    {
      std::unordered_set<std::string> tmp = property_graph_->getDisjoint(it);
      disjoint.insert(tmp.begin(), tmp.end());
    }

    std::string intersection = findIntersection(up, disjoint);
    if(intersection != "")
      print_error("'" + graph_vect_[i]->value() + "' can't be a '" + intersection + "' because of disjonction");
  }
}

} // namespace ontologenius
