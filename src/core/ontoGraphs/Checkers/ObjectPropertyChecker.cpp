#include "ontologenius/core/ontoGraphs/Checkers/ObjectPropertyChecker.h"

#include <cstddef>
#include <shared_mutex>
#include <unordered_set>

#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

  size_t ObjectPropertyChecker::check()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);

    checkDisjoint();
    checkCharacteristics();
    removeLoops();

    is_analysed = true;
    printStatus();

    return getErrors();
  }

  void ObjectPropertyChecker::checkDisjoint()
  {
    for(auto* property : graph_vect_)
    {
      std::unordered_set<ObjectPropertyBranch*> up;
      property_graph_->getUpPtr(property, up);

      auto* intersection = property_graph_->isDisjoint(up, up);
      if(intersection != nullptr)
      {
        ObjectPropertyBranch* disjoint_with = property_graph_->firstIntersection(up, intersection->disjoints_);

        if(disjoint_with != nullptr)
          printError("'" + property->value() + "' can't be a '" + intersection->value() + "' and a '" + disjoint_with->value() + "' because of disjonction between properties '" + intersection->value() + "' and '" + disjoint_with->value() + "'");
        else
          printError("'" + property->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
      }
    }
  }

  void ObjectPropertyChecker::checkCharacteristics()
  {
    for(auto* property : graph_vect_)
    {
      const Properties_t properties = property->properties_;

      if(properties.symetric_property_ && properties.antisymetric_property_)
        printError("'" + property->value() + "' can't be a 'symetric' and 'antisymetric'");

      if(properties.reflexive_property_ && properties.irreflexive_property_)
        printError("'" + property->value() + "' can't be a 'reflexive' and 'irreflexive'");
    }
  }

  void ObjectPropertyChecker::removeLoops()
  {
    for(auto* property : graph_vect_)
    {
      if(property->properties_.transitive_property_)
      {
        for(auto& inverse : property->inverses_)
        {
          if(inverse.elem->properties_.transitive_property_)
            inverse.elem->properties_.transitive_property_ = false;
        }
      }
    }
  }

} // namespace ontologenius
