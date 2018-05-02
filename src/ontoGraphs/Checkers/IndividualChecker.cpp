#include "ontoloGenius/ontoGraphs/Checkers/IndividualChecker.h"

size_t IndividualChecker::check()
{
  checkSame();

  checkReflexive();

  checkDomain();

  checkRange();

  checkAssymetric();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void IndividualChecker::checkSame()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    std::set<std::string> same = individual_graph_->getSame(graph_[i]->value_);
    std::set<std::string> distinct;

    std::set<std::string>::iterator it;
    for (it = same.begin(); it != same.end(); it++)
    {
      std::set<std::string> tmp = individual_graph_->getDistincts((std::string&)*it);
      distinct.insert(tmp.begin(), tmp.end());
    }

    std::string intersection = findIntersection(same, distinct);
    if(intersection != "")
      print_error("'" + graph_[i]->value_ + "' can't be same and distinct with '" + intersection + "'");
  }
}

void IndividualChecker::checkReflexive()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < graph_[i]->object_properties_name_.size(); prop_i++)
    {
      if(graph_[i]->object_properties_name_[prop_i]->properties_.reflexive_property_)
        if(graph_[i]->value_ != graph_[i]->object_properties_on_[prop_i]->value_)
          print_error("'" + graph_[i]->object_properties_name_[prop_i]->value_ + "' is reflexive so can't be from '" + graph_[i]->value_ + "' to '" + graph_[i]->object_properties_on_[prop_i]->value_ + "'");
      else if(graph_[i]->object_properties_name_[prop_i]->properties_.irreflexive_property_)
        if(graph_[i]->value_ == graph_[i]->object_properties_on_[prop_i]->value_)
          print_error("'" + graph_[i]->object_properties_name_[prop_i]->value_ + "' is irreflexive so can't be from '" + graph_[i]->value_ + "' to '" + graph_[i]->object_properties_on_[prop_i]->value_ + "'");
    }
  }
}

void IndividualChecker::checkDomain()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    std::set<std::string> up = individual_graph_->getUp(graph_[i]->value_);

    for(size_t prop_i = 0; prop_i < graph_[i]->object_properties_name_.size(); prop_i++)
    {
      std::set<std::string> domain = individual_graph_->object_property_graph_->getDomain(graph_[i]->object_properties_name_[prop_i]->value_);
      if(domain.size() != 0)
      {
        std::string intersection = findIntersection(up, domain);
        if(intersection == "")
          print_error("'" + graph_[i]->value_ + "' is not in domain of '" + graph_[i]->object_properties_name_[prop_i]->value_ + "'");
      }
    }
  }
}

void IndividualChecker::checkRange()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < graph_[i]->object_properties_name_.size(); prop_i++)
    {
      std::set<std::string> up = individual_graph_->getUp(graph_[i]->object_properties_on_[prop_i]->value_);
      std::set<std::string> range = individual_graph_->object_property_graph_->getRange(graph_[i]->object_properties_name_[prop_i]->value_);
      if(range.size() != 0)
      {
        std::string intersection = findIntersection(up, range);
        if(intersection == "")
          print_error("'" + graph_[i]->object_properties_on_[prop_i]->value_ + "' is not in range of '" + graph_[i]->object_properties_name_[prop_i]->value_ + "'");
      }
    }
  }
}

void IndividualChecker::checkAssymetric()
{
  for(size_t i = 0; i < graph_.size(); i++)
  {
    for(size_t prop_i = 0; prop_i < graph_[i]->object_properties_name_.size(); prop_i++)
    {
      if(graph_[i]->object_properties_name_[prop_i]->properties_.antisymetric_property_)
        if(symetricExist(graph_[i], graph_[i]->object_properties_name_[prop_i], graph_[i]->object_properties_on_[prop_i]))
          print_error("'" + graph_[i]->object_properties_name_[prop_i]->value_ + "' is antisymetric so can't be from '" + graph_[i]->value_ + "' to '" + graph_[i]->object_properties_on_[prop_i]->value_ + "' and inverse");
    }
  }
}

bool IndividualChecker::symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  for(size_t i = 0; i < sym_indiv->object_properties_name_.size(); i++)
  {
    if(sym_indiv->object_properties_name_[i]->value_ == sym_prop->value_)
      if(sym_indiv->object_properties_on_[i]->value_ == indiv_on->value_)
        return true;
  }
  return false;
}
