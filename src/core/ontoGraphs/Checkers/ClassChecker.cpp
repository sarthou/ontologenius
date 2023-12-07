#include "ontologenius/core/ontoGraphs/Checkers/ClassChecker.h"

#include <algorithm>

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

size_t ClassChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
  graph_size = graph_vect_.size();

  checkDisjoint();

  checkObjectPropertyDomain();
  checkObjectPropertyRange();

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

    ClassBranch_t* intersection = nullptr;
    ClassBranch_t* disjoint_with = nullptr;
    for(ClassBranch_t* it : up)
    {
      ClassBranch_t* tmp_intersection = class_graph_->firstIntersection(up, it->disjoints_);
      if(tmp_intersection != nullptr)
      {
        intersection = tmp_intersection;
        disjoint_with = class_graph_->firstIntersection(up, intersection->disjoints_);
        if(disjoint_with != nullptr)
          break;
      }
    }

    if(intersection != nullptr)
    {
      if(disjoint_with != nullptr)
        print_error("'" + branch->value() + "' can't be a '" + intersection->value() + "' and a '"
        + disjoint_with->value() + "' because of disjonction between classes '"
        + intersection->value() + "' and '" + disjoint_with->value() + "'");
      else
        print_error("'" + branch->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
    }
  }
}

void ClassChecker::checkObjectPropertyDomain()
{
  for(auto& _class : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(_class, up);

    std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

    for(ClassObjectRelationElement_t& object_relation : _class->object_relations_)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      class_graph_->object_property_graph_->getUpPtr(object_relation.first, prop_up);
      std::unordered_set<ClassBranch_t*> domain;
      for(auto prop : prop_up)
        class_graph_->object_property_graph_->getDomainPtr(prop, domain);

      if(domain.size() != 0)
      {
        ClassBranch_t* intersection = class_graph_->firstIntersection(up, domain);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto dom : domain)
            class_graph_->getDisjoint(dom, disjoints);
          intersection = class_graph_->firstIntersection(up, disjoints);

          if(intersection == nullptr)
          {
            _class->flags_["domain"].push_back(object_relation.first->value());
            print_warning("'" + _class->value() + "' is not in domain of '" + object_relation.first->value() + "'");
          }
          else
            print_error("'" + _class->value() + "' can not be in domain of '" + object_relation.first->value() + "'");
        }
      }
    }
  }
}

void ClassChecker::checkObjectPropertyRange()
{
  for(auto& _class : graph_vect_)
  {
    for(ClassObjectRelationElement_t& object_relation : _class->object_relations_)
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
        ClassBranch_t* intersection = class_graph_->firstIntersection(up, range);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto ran : range)
            class_graph_->getDisjoint(ran, disjoints);
          intersection = class_graph_->firstIntersection(up, disjoints);

          if(intersection == nullptr)
          {
            _class->flags_["range"].push_back(object_relation.first->value());
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
  for(auto& _class : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(_class, up);

    std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

    for(ClassDataRelationElement_t& relation : _class->data_relations_)
    {
      std::unordered_set<DataPropertyBranch_t*> prop_up;
      class_graph_->data_property_graph_->getUpPtr(relation.first, prop_up);
      std::unordered_set<ClassBranch_t*> domain;
      for(auto prop : prop_up)
        class_graph_->data_property_graph_->getDomainPtr(prop, domain);

      if(domain.size() != 0)
      {
        ClassBranch_t* intersection = class_graph_->firstIntersection(up, domain);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto dom : domain)
            class_graph_->getDisjoint(dom, disjoints);
          intersection = class_graph_->firstIntersection(up, disjoints);

          if(intersection == nullptr)
          {
            _class->flags_["range"].push_back(relation.first->value());
            print_warning("'" + _class->value() + "' is not in domain of '" + relation.first->value() + "'");
          }
          else
            print_error("'" + _class->value() + "' can not be in domain of '" + relation.first->value() + "'");
        }
      }
    }
  }
}

void ClassChecker::checkDataPropertyRange()
{
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
  for(auto& _class : graph_vect_)
  {
    for(ClassDataRelationElement_t& relation : _class->data_relations_)
    {
      std::unordered_set<std::string> range = class_graph_->data_property_graph_->getRange(relation.first->value());
      if(range.size() != 0)
      {
        auto intersection = range.find(relation.second->type_);
        if(intersection == range.end())
          print_error("'" + relation.second->type_ + "' is not in range of '" + relation.first->value() + "'");
      }
    }
  }
}

} // namespace ontologenius
