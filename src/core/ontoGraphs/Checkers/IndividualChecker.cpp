#include "ontoloGenius/core/ontoGraphs/Checkers/IndividualChecker.h"
#include <algorithm>

size_t IndividualChecker::check()
{
  graph_size = graph_.size();
  checkSame();

  checkReflexive();

  checkObectPropertyDomain();
  checkObectPropertyRange();

  checkDataPropertyDomain();
  checkDataPropertyRange();

  checkAssymetric();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void IndividualChecker::checkSame()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<std::string> same = individual_graph_->getSame(graph_[i]->value());
    std::unordered_set<std::string> distinct;

    for (std::string it : same)
    {
      std::unordered_set<std::string> tmp = individual_graph_->getDistincts(it);
      distinct.insert(tmp.begin(), tmp.end());
    }

    std::string intersection = findIntersection(same, distinct);
    if(intersection != "")
      print_error("'" + graph_[i]->value() + "' can't be same and distinct with '" + intersection + "'");
  }
}

void IndividualChecker::checkReflexive()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_[i]->object_properties_name_.size(); prop_i++)
    {
      if(graph_[i]->object_properties_name_[prop_i]->properties_.reflexive_property_)
      {
        if(graph_[i]->value() != graph_[i]->object_properties_on_[prop_i]->value())
          print_error("'" + graph_[i]->object_properties_name_[prop_i]->value() + "' is reflexive so can't be from '" + graph_[i]->value() + "' to '" + graph_[i]->object_properties_on_[prop_i]->value() + "'");
      }
      else if(graph_[i]->object_properties_name_[prop_i]->properties_.irreflexive_property_)
      {
        if(graph_[i]->value() == graph_[i]->object_properties_on_[prop_i]->value())
          print_error("'" + graph_[i]->object_properties_name_[prop_i]->value() + "' is irreflexive so can't be from '" + graph_[i]->value() + "' to '" + graph_[i]->object_properties_on_[prop_i]->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkObectPropertyDomain()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<std::string> up = individual_graph_->getUp(graph_[i]->value());

    for(size_t prop_i = 0; prop_i < graph_[i]->object_properties_name_.size(); prop_i++)
    {
      std::unordered_set<std::string> domain = individual_graph_->object_property_graph_->getDomain(graph_[i]->object_properties_name_[prop_i]->value());
      if(domain.size() != 0)
      {
        std::string intersection = findIntersection(up, domain);
        if(intersection == "")
          print_error("'" + graph_[i]->value() + "' is not in domain of '" + graph_[i]->object_properties_name_[prop_i]->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkObectPropertyRange()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_[i]->object_properties_name_.size(); prop_i++)
    {
      std::unordered_set<std::string> up = individual_graph_->getUp(graph_[i]->object_properties_on_[prop_i]->value());
      std::unordered_set<std::string> range = individual_graph_->object_property_graph_->getRange(graph_[i]->object_properties_name_[prop_i]->value());
      if(range.size() != 0)
      {
        std::string intersection = findIntersection(up, range);
        if(intersection == "")
          print_error("'" + graph_[i]->object_properties_on_[prop_i]->value() + "' is not in range of '" + graph_[i]->object_properties_name_[prop_i]->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkDataPropertyDomain()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<std::string> up = individual_graph_->getUp(graph_[i]->value());

    for(size_t prop_i = 0; prop_i < graph_[i]->data_properties_name_.size(); prop_i++)
    {
      std::unordered_set<std::string> domain = individual_graph_->data_property_graph_->getDomain(graph_[i]->data_properties_name_[prop_i]->value());
      if(domain.size() != 0)
      {
        std::string intersection = findIntersection(up, domain);
        if(intersection == "")
          print_error("'" + graph_[i]->value() + "' is not in domain of '" + graph_[i]->data_properties_name_[prop_i]->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkDataPropertyRange()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_[i]->data_properties_name_.size(); prop_i++)
    {
      std::unordered_set<std::string> range = individual_graph_->data_property_graph_->getRange(graph_[i]->data_properties_name_[prop_i]->value());
      if(range.size() != 0)
      {
        std::unordered_set<std::string>::iterator intersection = std::find(range.begin(), range.end(), graph_[i]->data_properties_data_[prop_i].type_);
        if(intersection == range.end())
          print_error("'" + graph_[i]->data_properties_data_[prop_i].type_ + "' is not in range of '" + graph_[i]->data_properties_name_[prop_i]->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkAssymetric()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_[i]->object_properties_name_.size(); prop_i++)
    {
      if(graph_[i]->object_properties_name_[prop_i]->properties_.antisymetric_property_)
        if(symetricExist(graph_[i], graph_[i]->object_properties_name_[prop_i], graph_[i]->object_properties_on_[prop_i]))
          print_error("'" + graph_[i]->object_properties_name_[prop_i]->value() + "' is antisymetric so can't be from '" + graph_[i]->value() + "' to '" + graph_[i]->object_properties_on_[prop_i]->value() + "' and inverse");
    }
  }
}

bool IndividualChecker::symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  for(size_t i = 0; i < sym_indiv->object_properties_name_.size(); i++)
  {
    if(sym_indiv->object_properties_name_[i]->value() == sym_prop->value())
      if(sym_indiv->object_properties_on_[i]->value() == indiv_on->value())
        return true;
  }
  return false;
}
