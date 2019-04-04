#include "ontoloGenius/core/ontoGraphs/Checkers/ClassChecker.h"
#include <algorithm>

#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

namespace ontologenius {

size_t ClassChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
  graph_size = graph_vect_.size();

  checkDisjoint();

  checkObectPropertyDomain();
  checkObectPropertyRange();

  checkDataPropertyDomain();
  checkDataPropertyRange();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void ClassChecker::checkDisjoint()
{
  for(ClassBranch_t* branch : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(branch, up);

    std::unordered_set<ClassBranch_t*> disjoint;

    for(ClassBranch_t* it : up)
    {
      for(ClassBranch_t* dis_i : it->disjoints_)
        class_graph_->getDownPtr(dis_i, disjoint);
    }
    // same as : class_graph_->getDisjoint(it, disjoint);

    ClassBranch_t* intersection = findIntersection(up, disjoint);
    if(intersection != nullptr)
      print_error("'" + branch->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
  }
}

void ClassChecker::checkObectPropertyDomain()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(graph_vect_[i], up);

    std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

    for(size_t prop_i = 0; prop_i < graph_vect_[i]->object_properties_name_.size(); prop_i++)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      class_graph_->object_property_graph_->getUpPtr(graph_vect_[i]->object_properties_name_[prop_i], prop_up);
      std::unordered_set<ClassBranch_t*> domain;
      for(auto prop : prop_up)
        class_graph_->object_property_graph_->getDomainPtr(prop, domain);

      if(domain.size() != 0)
      {
        ClassBranch_t* intersection = findIntersection(up, domain);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto dom : domain)
            class_graph_->getDisjoint(dom, disjoints);
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

void ClassChecker::checkObectPropertyRange()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_vect_[i]->object_properties_name_.size(); prop_i++)
    {
      std::unordered_set<ClassBranch_t*> up;
      class_graph_->getUpPtr(graph_vect_[i]->object_properties_on_[prop_i], up);

      std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      class_graph_->object_property_graph_->getUpPtr(graph_vect_[i]->object_properties_name_[prop_i], prop_up);
      std::unordered_set<ClassBranch_t*> range;
      for(auto prop : prop_up)
        class_graph_->object_property_graph_->getRangePtr(prop, range);

      if(range.size() != 0)
      {
        ClassBranch_t* intersection = findIntersection(up, range);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto ran : range)
            class_graph_->getDisjoint(ran, disjoints);
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

void ClassChecker::checkDataPropertyDomain()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(graph_vect_[i], up);

    std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

    for(size_t prop_i = 0; prop_i < graph_vect_[i]->data_properties_name_.size(); prop_i++)
    {
      std::unordered_set<DataPropertyBranch_t*> prop_up;
      class_graph_->data_property_graph_->getUpPtr(graph_vect_[i]->data_properties_name_[prop_i], prop_up);
      std::unordered_set<ClassBranch_t*> domain;
      for(auto prop : prop_up)
        class_graph_->data_property_graph_->getDomainPtr(prop, domain);

      if(domain.size() != 0)
      {
        ClassBranch_t* intersection = findIntersection(up, domain);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto dom : domain)
            class_graph_->getDisjoint(dom, disjoints);
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

void ClassChecker::checkDataPropertyRange()
{
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
  for(size_t i = 0; i < graph_size; i++)
  {
    for(size_t prop_i = 0; prop_i < graph_vect_[i]->data_properties_name_.size(); prop_i++)
    {
      std::unordered_set<std::string> range = class_graph_->data_property_graph_->getRange(graph_vect_[i]->data_properties_name_[prop_i]->value());
      if(range.size() != 0)
      {
        std::unordered_set<std::string>::iterator intersection = std::find(range.begin(), range.end(), graph_vect_[i]->data_properties_data_[prop_i].type_);
        if(intersection == range.end())
          print_error("'" + graph_vect_[i]->data_properties_data_[prop_i].type_ + "' is not in range of '" + graph_vect_[i]->data_properties_name_[prop_i]->value() + "'");
      }
    }
  }
}

} // namespace ontologenius
