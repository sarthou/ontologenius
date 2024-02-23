#include "ontologenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"

#include <unordered_set>

namespace ontologenius {

size_t DataPropertyChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);

  checkDisjoint();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void DataPropertyChecker::checkDisjoint()
{
  for(auto property : graph_vect_)
  {
    std::unordered_set<DataPropertyBranch_t*> up;
    property_graph_->getUpPtr(property, up);

    auto intersection = property_graph_->isDisjoint(up, up);
    if(intersection != nullptr)
    {
      DataPropertyBranch_t* disjoint_with = property_graph_->firstIntersection(up, intersection->disjoints_);

      if(disjoint_with != nullptr)
        print_error("'" + property->value() + "' can't be a '" + intersection->value() + "' and a '"
        + disjoint_with->value() + "' because of disjonction between properties '"
        + intersection->value() + "' and '" + disjoint_with->value() + "'");
      else
        print_error("'" + property->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
    }
  }
}

} // namespace ontologenius
