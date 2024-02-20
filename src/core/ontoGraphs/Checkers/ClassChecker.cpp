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

    auto intersection = class_graph_->isDisjoint(up, up);
    if(intersection != nullptr)
    {
      ClassBranch_t* disjoint_with = class_graph_->firstIntersection(up, intersection->disjoints_);

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
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
  for(auto& _class : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(_class, up);

    for(ClassObjectRelationElement_t& object_relation : _class->object_relations_)
    {
      std::unordered_set<ClassBranch_t*> domain;
      class_graph_->object_property_graph_->getDomainPtr(object_relation.first, domain, 0);

      if(domain.size() != 0)
      {
        auto intersection = class_graph_->checkDomainOrRange(domain, up);
        if(intersection.first == false)
        {
          if(intersection.second == nullptr)
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
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

  for(auto& _class : graph_vect_)
  {
    for(ClassObjectRelationElement_t& object_relation : _class->object_relations_)
    {
      std::unordered_set<ClassBranch_t*> range;
      class_graph_->object_property_graph_->getRangePtr(object_relation.first, range, 0);
     
      if(range.size() != 0)
      {
        std::unordered_set<ClassBranch_t*> up;
        class_graph_->getUpPtr(object_relation.second, up);

        auto intersection = class_graph_->checkDomainOrRange(range, up);
        if(intersection.first == false)
        {
          if(intersection.second == nullptr)
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
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

  for(auto& _class : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(_class, up);

    for(ClassDataRelationElement_t& relation : _class->data_relations_)
    {
      std::unordered_set<ClassBranch_t*> domain;
      class_graph_->data_property_graph_->getDomainPtr(relation.first, domain, 0);

      if(domain.size() != 0)
      {
        auto intersection = class_graph_->checkDomainOrRange(domain, up);
        if(intersection.first == false)
        {
          if(intersection.second == nullptr)
          {
            _class->flags_["domain"].push_back(relation.first->value());
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
