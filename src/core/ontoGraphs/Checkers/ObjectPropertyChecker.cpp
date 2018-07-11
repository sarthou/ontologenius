#include "ontoloGenius/core/ontoGraphs/Checkers/ObjectPropertyChecker.h"
#include <unordered_set>

size_t ObjectPropertyChecker::check()
{
  graph_size = graph_.size();
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

void ObjectPropertyChecker::checkCharacteristics()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    Properties_t properties = graph_[i]->properties_;

    if(properties.symetric_property_ && properties.antisymetric_property_)
      print_error("'" + graph_[i]->value() + "' can't be a 'symetric' and 'antisymetric'");

    if(properties.reflexive_property_ && properties.irreflexive_property_)
      print_error("'" + graph_[i]->value() + "' can't be a 'reflexive' and 'irreflexive'");
  }
}
