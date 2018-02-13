#include "ontoloGenius/ontoGraphs/Checkers/PropertyChecker.h"
#include <set>

size_t PropertyChecker::check()
{
  checkDisjoint();
  checkCharacteristics();

  printStatus("Property");
}

void PropertyChecker::checkDisjoint()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    std::set<std::string> up = property_graph_->getUp(graph_[i]->value_);
    std::set<std::string> disjoint;

    std::set<std::string>::iterator it;
    for (it = up.begin(); it != up.end(); it++)
    {
      std::set<std::string> tmp = property_graph_->getDisjoint(*it);
      disjoint.insert(tmp.begin(), tmp.end());
    }

    std::string intersection = findIntersection(up, disjoint);
    if(intersection != "")
      print_error("'" + graph_[i]->value_ + "' can't be a '" + intersection + "' because of disjonction");
  }
}

void PropertyChecker::checkCharacteristics()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    Properties_t properties = graph_[i]->properties_;

    if(properties.symetric_property_ && properties.antisymetric_property_)
      print_error("'" + graph_[i]->value_ + "' can't be a 'symetric' and 'antisymetric'");

    if(properties.reflexive_property_ && properties.irreflexive_property_)
      print_error("'" + graph_[i]->value_ + "' can't be a 'reflexive' and 'irreflexive'");
  }
}
