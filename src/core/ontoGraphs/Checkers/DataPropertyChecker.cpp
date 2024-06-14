#include "ontologenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"

#include <cstddef>
#include <shared_mutex>
#include <unordered_set>

#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

namespace ontologenius {

  size_t DataPropertyChecker::check()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);

    checkDisjoint();

    is_analysed = true;
    printStatus();

    return getErrors();
  }

  void DataPropertyChecker::checkDisjoint()
  {
    for(auto* property : graph_vect_)
    {
      std::unordered_set<DataPropertyBranch*> up;
      property_graph_->getUpPtr(property, up);

      auto* intersection = property_graph_->isDisjoint(up, up);
      if(intersection != nullptr)
      {
        DataPropertyBranch* disjoint_with = property_graph_->firstIntersection(up, intersection->disjoints_);

        if(disjoint_with != nullptr)
          printError("'" + property->value() + "' can't be a '" + intersection->value() + "' and a '" + disjoint_with->value() + "' because of disjonction between properties '" + intersection->value() + "' and '" + disjoint_with->value() + "'");
        else
          printError("'" + property->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
      }
    }
  }

} // namespace ontologenius
