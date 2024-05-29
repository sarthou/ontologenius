#include "ontologenius/core/ontoGraphs/Checkers/ClassChecker.h"

#include <algorithm>
#include <cstddef>
#include <shared_mutex>
#include <unordered_set>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

  size_t ClassChecker::check()
  {
    std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
    std::unordered_set<ClassBranch*> up;

    for(auto& class_check : graph_vect_)
    {
      class_graph_->getUpPtr(class_check, up);

      checkDisjoint(class_check, up);
      checkObjectPropertyDomain(class_check, up);
      checkObjectPropertyRange(class_check);

      checkDataPropertyDomain(class_check, up);
      checkDataPropertyRange(class_check);

      up.clear();
    }

    is_analysed = true;
    printStatus();

    return getErrors();
  }

  void ClassChecker::checkDisjoint(ClassBranch* branch, std::unordered_set<ClassBranch*> up)
  {
    auto* intersection = class_graph_->isDisjoint(up, up);
    if(intersection != nullptr)
    {
      ClassBranch* disjoint_with = class_graph_->firstIntersection(up, intersection->disjoints_);

      if(disjoint_with != nullptr)
        print_error("'" + branch->value() + "' can't be a '" + intersection->value() + "' and a '" + disjoint_with->value() + "' because of disjonction between classes '" + intersection->value() + "' and '" + disjoint_with->value() + "'");
      else
        print_error("'" + branch->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
    }
  }

  void ClassChecker::checkObjectPropertyDomain(ClassBranch* branch, std::unordered_set<ClassBranch*> up)
  {
    for(const ClassObjectRelationElement& object_relation : branch->object_relations_)
    {
      std::unordered_set<ClassBranch*> domain;
      class_graph_->object_property_graph_->getDomainPtr(object_relation.first, domain, 0);

      if(domain.empty() == false)
      {
        auto intersection = class_graph_->checkDomainOrRange(domain, up);
        if(intersection.first == false)
        {
          if(intersection.second == nullptr)
          {
            branch->flags_["domain"].push_back(object_relation.first->value());
            print_warning("'" + branch->value() + "' is not in domain of '" + object_relation.first->value() + "'");
          }
          else
            print_error("'" + branch->value() + "' can not be in domain of '" + object_relation.first->value() + "'");
        }
      }
    }
  }

  void ClassChecker::checkObjectPropertyRange(ClassBranch* branch)
  {
    for(const ClassObjectRelationElement& object_relation : branch->object_relations_)
    {
      std::unordered_set<ClassBranch*> range;
      class_graph_->object_property_graph_->getRangePtr(object_relation.first, range, 0);

      if(range.empty() == false)
      {
        std::unordered_set<ClassBranch*> up;
        class_graph_->getUpPtr(object_relation.second, up);

        auto intersection = class_graph_->checkDomainOrRange(range, up);
        if(intersection.first == false)
        {
          if(intersection.second == nullptr)
          {
            branch->flags_["range"].push_back(object_relation.first->value());
            print_warning("'" + object_relation.second->value() + "' is not in range of '" + object_relation.first->value() + "'");
          }
          else
            print_error("'" + object_relation.second->value() + "' can not be in range of '" + object_relation.first->value() + "'");
        }
      }
    }
  }

  void ClassChecker::checkDataPropertyDomain(ClassBranch* branch, std::unordered_set<ClassBranch*> up)
  {
    for(const ClassDataRelationElement& relation : branch->data_relations_)
    {
      std::unordered_set<ClassBranch*> domain;
      class_graph_->data_property_graph_->getDomainPtr(relation.first, domain, 0);

      if(domain.empty() == false)
      {
        auto intersection = class_graph_->checkDomainOrRange(domain, up);
        if(intersection.first == false)
        {
          if(intersection.second == nullptr)
          {
            branch->flags_["domain"].push_back(relation.first->value());
            print_warning("'" + branch->value() + "' is not in domain of '" + relation.first->value() + "'");
          }
          else
            print_error("'" + branch->value() + "' can not be in domain of '" + relation.first->value() + "'");
        }
      }
    }
  }

  void ClassChecker::checkDataPropertyRange(ClassBranch* branch)
  {
    for(const ClassDataRelationElement& relation : branch->data_relations_)
    {
      std::unordered_set<std::string> range = class_graph_->data_property_graph_->getRange(relation.first->value());
      if(range.empty() == false)
      {
        auto intersection = range.find(relation.second->type_);
        if(intersection == range.end())
          print_error("'" + relation.second->type_ + "' is not in range of '" + relation.first->value() + "'");
      }
    }
  }

} // namespace ontologenius
