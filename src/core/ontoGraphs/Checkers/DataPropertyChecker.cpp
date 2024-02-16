#include "ontologenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"

#include <unordered_set>

namespace ontologenius {

size_t DataPropertyChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);
  graph_size = graph_vect_.size();

  checkDisjoint();
  // TODO check domains and ranges possible intersection

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
      print_error("'" + property->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
  }
}

} // namespace ontologenius
