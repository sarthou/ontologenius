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

    std::unordered_set<ClassBranch_t*> disjoints;

    for(ClassBranch_t* it : up)
    {
      for(auto& disjoint : it->disjoints_)
        class_graph_->getDownPtr(disjoint.elem, disjoints);
    }

    ClassBranch_t* intersection = findIntersection(up, disjoints);
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

    for(ClassObjectRelationElement_t& object_relation : graph_vect_[i]->object_relations_)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      class_graph_->object_property_graph_->getUpPtr(object_relation.first, prop_up);
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
            graph_vect_[i]->flags_["domain"].push_back(object_relation.first->value());
            print_warning("'" + graph_vect_[i]->value() + "' is not in domain of '" + object_relation.first->value() + "'");
          }
          else
            print_error("'" + graph_vect_[i]->value() + "' can not be in domain of '" + object_relation.first->value() + "'");
        }
      }
    }
  }
}

void ClassChecker::checkObectPropertyRange()
{
  for(size_t i = 0; i < graph_size; i++)
  {
    for(ClassObjectRelationElement_t& object_relation : graph_vect_[i]->object_relations_)
    {
      std::unordered_set<ClassBranch_t*> up;
      class_graph_->getUpPtr(object_relation.second, up);

      std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      class_graph_->object_property_graph_->getUpPtr(object_relation.first, prop_up);
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
            graph_vect_[i]->flags_["range"].push_back(object_relation.first->value());
            print_warning("'" + object_relation.second->value() + "' is not in range of '" + object_relation.first->value() + "'");
          }
          else
            print_error("'" + object_relation.second->value() + "' can not be in range of '" + object_relation.first->value() + "'");
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

    for(ClassDataRelationElement_t& relation : graph_vect_[i]->data_relations_)
    {
      std::unordered_set<DataPropertyBranch_t*> prop_up;
      class_graph_->data_property_graph_->getUpPtr(relation.first, prop_up);
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
            graph_vect_[i]->flags_["range"].push_back(relation.first->value());
            print_warning("'" + graph_vect_[i]->value() + "' is not in domain of '" + relation.first->value() + "'");
          }
          else
            print_error("'" + graph_vect_[i]->value() + "' can not be in domain of '" + relation.first->value() + "'");
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
    for(ClassDataRelationElement_t& relation : graph_vect_[i]->data_relations_)
    {
      std::unordered_set<std::string> range = class_graph_->data_property_graph_->getRange(relation.first->value());
      if(range.size() != 0)
      {
        std::unordered_set<std::string>::iterator intersection = std::find(range.begin(), range.end(), relation.second.type_);
        if(intersection == range.end())
          print_error("'" + relation.second.type_ + "' is not in range of '" + relation.first->value() + "'");
      }
    }
  }
}

} // namespace ontologenius
