#include "ontologenius/core/ontoGraphs/Checkers/IndividualChecker.h"

#include <algorithm>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

size_t IndividualChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  std::unordered_set<ClassBranch_t*> ups;

  for(IndividualBranch_t* indiv : graph_vect_)
  {
    individual_graph_->getUpPtr(indiv, ups);

    checkDisjointInheritance(indiv, ups);
    checkDisjoint(indiv);
    checkReflexive(indiv);
    checkObectRelations(indiv, ups);
    checkDataRelations(indiv, ups);
    checkAssymetric(indiv);

    ups.clear();
  }

  is_analysed = true;
  printStatus();

  return getErrors();
}

void IndividualChecker::checkDisjointInheritance(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*> ups)
{
  ClassBranch_t* intersection = nullptr;
  ClassBranch_t* disjoint_with = nullptr;
  for(ClassBranch_t* it : ups)
  {
    ClassBranch_t* tmp_intersection = individual_graph_->class_graph_->firstIntersection(ups, it->disjoints_);
    if(tmp_intersection != nullptr)
    {
      intersection = tmp_intersection;
      disjoint_with = individual_graph_->class_graph_->firstIntersection(ups, intersection->disjoints_);
      if(disjoint_with != nullptr)
        break;
    }
  }

  if(intersection != nullptr)
  {
    if(disjoint_with != nullptr)
      print_error("'" + indiv->value() + "' can't be a '" + intersection->value() + "' and a '"
      + disjoint_with->value() + "' because of disjonction between classes '"
      + intersection->value() + "' and '" + disjoint_with->value() + "'");
    else
      print_error("'" + indiv->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
  }
}

void IndividualChecker::checkDisjoint(IndividualBranch_t* indiv)
{
  if(indiv->same_as_.size())
  {
    std::unordered_set<IndividualBranch_t*> sames;
    individual_graph_->getSame(indiv, sames);
    for(auto& same : indiv->same_as_)
    {
      IndividualBranch_t* intersection = individual_graph_->firstIntersection(sames, same.elem->distinct_);
      if(intersection != nullptr)
      {
        if(same.elem == intersection)
          continue;
        else if(indiv == intersection)
          continue;
        else if(indiv == same.elem)
          print_error("'" + indiv->value() + "' can't be same and distinct with '" + intersection->value() + "'");
        else
          print_error("'" + indiv->value() + "' can't be same as '" + same.elem->value() + "' and '" + intersection->value() + "' as they are distinct");
      }
    }
  }
}

void IndividualChecker::checkReflexive(IndividualBranch_t* indiv)
{
  for(IndivObjectRelationElement_t& object_relation : indiv->object_relations_)
  {
    if(object_relation.first->properties_.reflexive_property_)
    {
      if(indiv->get() != object_relation.second->get())
        print_error("'" + object_relation.first->value() + "' is reflexive so can't be from '" + indiv->value() + "' to '" + object_relation.second->value() + "'");
    }
    else if(object_relation.first->properties_.irreflexive_property_)
    {
      if(indiv->get() == object_relation.second->get())
        print_error("'" + object_relation.first->value() + "' is irreflexive so can't be from '" + indiv->value() + "' to '" + object_relation.second->value() + "'");
    }
  }
}

void IndividualChecker::checkObectRelations(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*> up_from)
{
  for(IndivObjectRelationElement_t& object_relation : indiv->object_relations_)
  {
    std::unordered_set<ClassBranch_t*> domain;
    std::unordered_set<ClassBranch_t*> range;
    individual_graph_->object_property_graph_->getDomainAndRangePtr(object_relation.first, domain, range, 0);

    if(domain.size() != 0)
    {
      auto intersection = individual_graph_->class_graph_->checkDomainOrRange(domain, up_from);
      if(intersection.first == false)
      {
        if(intersection.second == nullptr)
        {
          indiv->flags_["domain"].push_back(object_relation.first->value());
          print_warning("Individual '" + indiv->value() + "' is not in domain of object property '" + object_relation.first->value() + "'");
        }
        else
          print_error("Individual '" + indiv->value() + "' can not be in domain of object property '" + object_relation.first->value() + "'");
      }
    }

    if(range.size() != 0)
    {
      std::unordered_set<ClassBranch_t*> up_on;
      individual_graph_->getUpPtr(object_relation.second, up_on);

      auto intersection = individual_graph_->class_graph_->checkDomainOrRange(range, up_on);
      if(intersection.first == false)
      {
        if(intersection.second == nullptr)
        {
          indiv->flags_["range"].push_back(object_relation.first->value());
          print_warning("Individual '" + object_relation.second->value() + "' is not in range of object property '" + object_relation.first->value() + "'");
        }
        else
          print_error("Individual '" + object_relation.second->value() + "' can not be in range of object property '" + object_relation.first->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkDataRelations(IndividualBranch_t* indiv, std::unordered_set<ClassBranch_t*> up_from)
{
  for(IndivDataRelationElement_t& relation : indiv->data_relations_)
  {
    std::unordered_set<ClassBranch_t*> domain;
    individual_graph_->data_property_graph_->getDomainPtr(relation.first, domain, 0);

    if(domain.size() != 0)
    {
      auto intersection = individual_graph_->class_graph_->checkDomainOrRange(domain, up_from);
      if(intersection.first == false)
      {
        if(intersection.second == nullptr)
        {
          indiv->flags_["domain"].push_back(relation.first->value());
          print_warning("Individual '" + indiv->value() + "' is not in domain of data property '" + relation.first->value() + "'");
        }
        else
          print_error("Individual '" + indiv->value() + "' can not be in domain of data property '" + relation.first->value() + "'");
      }
    }

    std::unordered_set<std::string> range = individual_graph_->data_property_graph_->getRange(relation.first->value());
    if(range.size() != 0)
    {
      auto intersection = range.find(relation.second->type_);
      if(intersection == range.end())
        print_error("Individual '" + relation.second->type_ + "' is not in range of '" + relation.first->value() + "'");
    }
  }
}

void IndividualChecker::checkAssymetric(IndividualBranch_t* indiv)
{
  for(IndivObjectRelationElement_t& object_relation : indiv->object_relations_)
  {
    if(object_relation.first->properties_.antisymetric_property_)
      if(symetricExist(indiv, object_relation.first, object_relation.second))
        print_error("'" + object_relation.first->value() + "' is antisymetric so can't be from '" + indiv->value() + "' to '" + object_relation.second->value() + "' and inverse");
  }
}

bool IndividualChecker::symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  for(IndivObjectRelationElement_t& object_relation : sym_indiv->object_relations_)
  {
    if(object_relation.first->get() == sym_prop->get())
      if(object_relation.second->get() == indiv_on->get())
        return true;
  }
  return false;
}

} // namespace ontologenius
