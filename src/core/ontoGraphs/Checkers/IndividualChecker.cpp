#include "ontoloGenius/core/ontoGraphs/Checkers/IndividualChecker.h"
#include <algorithm>

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

size_t IndividualChecker::check()
{
  graph_size = graph_vect_.size();
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
    std::unordered_set<std::string> same = individual_graph_->getSame(graph_vect_[i]->value());
    std::unordered_set<std::string> distinct;

    for (std::string it : same)
    {
      std::unordered_set<std::string> tmp = individual_graph_->getDistincts(it);
      distinct.insert(tmp.begin(), tmp.end());
    }

    std::string intersection = findIntersection(same, distinct);
    if(intersection != "")
      print_error("'" + graph_vect_[i]->value() + "' can't be same and distinct with '" + intersection + "'");
  }
}

void IndividualChecker::checkReflexive()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_vect_[i]->object_properties_name_.size(); prop_i++)
    {
      if(graph_vect_[i]->object_properties_name_[prop_i]->properties_.reflexive_property_)
      {
        if(graph_vect_[i]->get() != graph_vect_[i]->object_properties_on_[prop_i]->get())
          print_error("'" + graph_vect_[i]->object_properties_name_[prop_i]->value() + "' is reflexive so can't be from '" + graph_vect_[i]->value() + "' to '" + graph_vect_[i]->object_properties_on_[prop_i]->value() + "'");
      }
      else if(graph_vect_[i]->object_properties_name_[prop_i]->properties_.irreflexive_property_)
      {
        if(graph_vect_[i]->get() == graph_vect_[i]->object_properties_on_[prop_i]->get())
          print_error("'" + graph_vect_[i]->object_properties_name_[prop_i]->value() + "' is irreflexive so can't be from '" + graph_vect_[i]->value() + "' to '" + graph_vect_[i]->object_properties_on_[prop_i]->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkObectPropertyDomain()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<ClassBranch_t*> up;
    individual_graph_->getUpPtr(graph_vect_[i], up);

    std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

    for(size_t prop_i = 0; prop_i < graph_vect_[i]->object_properties_name_.size(); prop_i++)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      individual_graph_->object_property_graph_->getUpPtr(graph_vect_[i]->object_properties_name_[prop_i], prop_up);
      std::unordered_set<ClassBranch_t*> domain;
      for(auto prop : prop_up)
        individual_graph_->object_property_graph_->getDomainPtr(prop, domain);

      if(domain.size() != 0)
      {
        ClassBranch_t* intersection = findIntersection(up, domain);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto dom : domain)
            individual_graph_->class_graph_->getDisjoint(dom, disjoints);
          intersection = findIntersection(up, disjoints);

          if(intersection == nullptr)
          {
            graph_vect_[i]->flags_["domain"].push_back(graph_vect_[i]->object_properties_name_[prop_i]->value());
            print_warning("'" + graph_vect_[i]->value() + "' is not in domain of '" + graph_vect_[i]->object_properties_name_[prop_i]->value() + "'");
          }
          else
            print_error("'" + graph_vect_[i]->value() + "' can not be in domain of '" + graph_vect_[i]->object_properties_name_[prop_i]->value() + "'");
        }
      }
    }
  }
}

void IndividualChecker::checkObectPropertyRange()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_vect_[i]->object_properties_name_.size(); prop_i++)
    {
      std::unordered_set<ClassBranch_t*> up;
      individual_graph_->getUpPtr(graph_vect_[i]->object_properties_on_[prop_i], up);

      std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      individual_graph_->object_property_graph_->getUpPtr(graph_vect_[i]->object_properties_name_[prop_i], prop_up);
      std::unordered_set<ClassBranch_t*> range;
      for(auto prop : prop_up)
        individual_graph_->object_property_graph_->getRangePtr(prop, range);

      if(range.size() != 0)
      {
        ClassBranch_t* intersection = findIntersection(up, range);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto ran : range)
            individual_graph_->class_graph_->getDisjoint(ran, disjoints);
          intersection = findIntersection(up, disjoints);

          if(intersection == nullptr)
          {
            graph_vect_[i]->flags_["range"].push_back(graph_vect_[i]->object_properties_name_[prop_i]->value());
            print_warning("'" + graph_vect_[i]->object_properties_on_[prop_i]->value() + "' is not in range of '" + graph_vect_[i]->object_properties_name_[prop_i]->value() + "'");
          }
          else
            print_error("'" + graph_vect_[i]->object_properties_on_[prop_i]->value() + "' can not be in range of '" + graph_vect_[i]->object_properties_name_[prop_i]->value() + "'");
        }
      }
    }
  }
}

void IndividualChecker::checkDataPropertyDomain()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<ClassBranch_t*> up;
    individual_graph_->getUpPtr(graph_vect_[i], up);

    std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

    for(size_t prop_i = 0; prop_i < graph_vect_[i]->data_properties_name_.size(); prop_i++)
    {
      std::unordered_set<DataPropertyBranch_t*> prop_up;
      individual_graph_->data_property_graph_->getUpPtr(graph_vect_[i]->data_properties_name_[prop_i], prop_up);
      std::unordered_set<ClassBranch_t*> domain;
      for(auto prop : prop_up)
        individual_graph_->data_property_graph_->getDomainPtr(prop, domain);

      if(domain.size() != 0)
      {
        ClassBranch_t* intersection = findIntersection(up, domain);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto dom : domain)
            individual_graph_->class_graph_->getDisjoint(dom, disjoints);
          intersection = findIntersection(up, disjoints);

          if(intersection == nullptr)
          {
            graph_vect_[i]->flags_["range"].push_back(graph_vect_[i]->data_properties_name_[prop_i]->value());
            print_warning("'" + graph_vect_[i]->value() + "' is not in domain of '" + graph_vect_[i]->data_properties_name_[prop_i]->value() + "'");
          }
          else
            print_error("'" + graph_vect_[i]->value() + "' can not be in domain of '" + graph_vect_[i]->data_properties_name_[prop_i]->value() + "'");
        }
      }
    }
  }
}

void IndividualChecker::checkDataPropertyRange()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_vect_[i]->data_properties_name_.size(); prop_i++)
    {
      std::unordered_set<std::string> range = individual_graph_->data_property_graph_->getRange(graph_vect_[i]->data_properties_name_[prop_i]->value());
      if(range.size() != 0)
      {
        std::unordered_set<std::string>::iterator intersection = std::find(range.begin(), range.end(), graph_vect_[i]->data_properties_data_[prop_i].type_);
        if(intersection == range.end())
          print_error("'" + graph_vect_[i]->data_properties_data_[prop_i].type_ + "' is not in range of '" + graph_vect_[i]->data_properties_name_[prop_i]->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkAssymetric()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_vect_[i]->object_properties_name_.size(); prop_i++)
    {
      if(graph_vect_[i]->object_properties_name_[prop_i]->properties_.antisymetric_property_)
        if(symetricExist(graph_vect_[i], graph_vect_[i]->object_properties_name_[prop_i], graph_vect_[i]->object_properties_on_[prop_i]))
          print_error("'" + graph_vect_[i]->object_properties_name_[prop_i]->value() + "' is antisymetric so can't be from '" + graph_vect_[i]->value() + "' to '" + graph_vect_[i]->object_properties_on_[prop_i]->value() + "' and inverse");
    }
  }
}

bool IndividualChecker::symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  for(size_t i = 0; i < sym_indiv->object_properties_name_.size(); i++)
  {
    if(sym_indiv->object_properties_name_[i]->get() == sym_prop->get())
      if(sym_indiv->object_properties_on_[i]->get() == indiv_on->get())
        return true;
  }
  return false;
}
