#include "ontoloGenius/core/ontoGraphs/Checkers/ObjectPropertyChecker.h"

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
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<ObjectPropertyBranch_t*> up;
    property_graph_->getUpPtr(graph_vect_[i], up);
    std::unordered_set<ObjectPropertyBranch_t*> disjoint;

    for(ObjectPropertyBranch_t* it : up)
      property_graph_->getDisjoint(it, disjoint);

    ObjectPropertyBranch_t* intersection = findIntersection(up, disjoint);
    if(intersection != nullptr)
      print_error("'" + graph_vect_[i]->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
  }
}

void ObjectPropertyChecker::checkCharacteristics()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    Properties_t properties = graph_vect_[i]->properties_;

    if(properties.symetric_property_ && properties.antisymetric_property_)
      print_error("'" + graph_vect_[i]->value() + "' can't be a 'symetric' and 'antisymetric'");

    if(properties.reflexive_property_ && properties.irreflexive_property_)
      print_error("'" + graph_vect_[i]->value() + "' can't be a 'reflexive' and 'irreflexive'");
  }
}

} // namespace ontologenius
