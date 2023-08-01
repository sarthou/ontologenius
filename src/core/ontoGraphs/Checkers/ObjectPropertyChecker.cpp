#include "ontologenius/core/ontoGraphs/Checkers/ObjectPropertyChecker.h"

#include <unordered_set>

namespace ontologenius {

size_t ObjectPropertyChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);
  graph_size = graph_vect_.size();

  checkDisjoint();
  checkCharacteristics();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void ObjectPropertyChecker::checkDisjoint()
{
  for(auto property : graph_vect_)
  {
    std::unordered_set<ObjectPropertyBranch_t*> up;
    property_graph_->getUpPtr(property, up);
    std::unordered_set<ObjectPropertyBranch_t*> disjoint;

    for(ObjectPropertyBranch_t* it : up)
      property_graph_->getDisjointPtr(it, disjoint);

    ObjectPropertyBranch_t* intersection = findIntersection(up, disjoint);
    if(intersection != nullptr)
      print_error("'" + property->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
  }
}

void ObjectPropertyChecker::checkCharacteristics()
{
  for(auto property : graph_vect_)
  {
    Properties_t properties = property->properties_;

    if(properties.symetric_property_ && properties.antisymetric_property_)
      print_error("'" + property->value() + "' can't be a 'symetric' and 'antisymetric'");

    if(properties.reflexive_property_ && properties.irreflexive_property_)
      print_error("'" + property->value() + "' can't be a 'reflexive' and 'irreflexive'");
  }
}

} // namespace ontologenius
