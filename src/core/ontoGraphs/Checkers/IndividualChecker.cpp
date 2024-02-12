#include "ontologenius/core/ontoGraphs/Checkers/IndividualChecker.h"

#include <algorithm>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

size_t IndividualChecker::check()
{
  graph_size = graph_vect_.size();

  checkDisjointInheritance();
  checkDisjoint();

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

void IndividualChecker::checkDisjointInheritance()
{
  for(IndividualBranch_t* indiv : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    individual_graph_->getUpPtr(indiv, up);

    ClassBranch_t* intersection = nullptr;
    ClassBranch_t* disjoint_with = nullptr;
    for(ClassBranch_t* it : up)
    {
      ClassBranch_t* tmp_intersection = individual_graph_->class_graph_->firstIntersection(up, it->disjoints_);
      if(tmp_intersection != nullptr)
      {
        intersection = tmp_intersection;
        disjoint_with = individual_graph_->class_graph_->firstIntersection(up, intersection->disjoints_);
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
}

void IndividualChecker::checkDisjoint()
{
  for(IndividualBranch_t* indiv : graph_vect_)
  {
    if(indiv->same_as_.size())
    {
      std::unordered_set<IndividualBranch_t*> sames;
      individual_graph_->getSame(indiv, sames);
      for(auto same : indiv->same_as_)
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
}

void IndividualChecker::checkReflexive()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  for(IndividualBranch_t* indiv : graph_vect_)
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
}

void IndividualChecker::checkObectPropertyDomain()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

  for(auto& indiv : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    individual_graph_->getUpPtr(indiv, up);

    for(IndivObjectRelationElement_t& object_relation : indiv->object_relations_)
    {
      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      individual_graph_->object_property_graph_->getUpPtr(object_relation.first, prop_up);
      std::unordered_set<ClassBranch_t*> domain;
      for(auto prop : prop_up)
        individual_graph_->object_property_graph_->getDomainPtr(prop, domain);

      if(domain.size() != 0)
      {
        ClassBranch_t* intersection = individual_graph_->class_graph_->firstIntersection(up, domain);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto dom : domain)
            individual_graph_->class_graph_->getDisjoint(dom, disjoints);
          intersection = individual_graph_->class_graph_->firstIntersection(up, disjoints);

          if(intersection == nullptr)
          {
            indiv->flags_["domain"].push_back(object_relation.first->value());
            print_warning("Individual '" + indiv->value() + "' is not in domain of object property '" + object_relation.first->value() + "'");
          }
          else
            print_error("Individual '" + indiv->value() + "' can not be in domain of object property '" + object_relation.first->value() + "'");
        }
      }
    }
  }
}

void IndividualChecker::checkObectPropertyRange()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

  for(auto& indiv : graph_vect_)
  {
    for(IndivObjectRelationElement_t& object_relation : indiv->object_relations_)
    {
      std::unordered_set<ClassBranch_t*> up;
      individual_graph_->getUpPtr(object_relation.second, up);

      std::unordered_set<ObjectPropertyBranch_t*> prop_up;
      individual_graph_->object_property_graph_->getUpPtr(object_relation.first, prop_up);
      std::unordered_set<ClassBranch_t*> range;
      for(auto prop : prop_up)
        individual_graph_->object_property_graph_->getRangePtr(prop, range);

      if(range.size() != 0)
      {
        ClassBranch_t* intersection = individual_graph_->class_graph_->firstIntersection(up, range);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto ran : range)
            individual_graph_->class_graph_->getDisjoint(ran, disjoints);
          intersection = individual_graph_->class_graph_->firstIntersection(up, disjoints);

          if(intersection == nullptr)
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
}

void IndividualChecker::checkDataPropertyDomain()
{
  for(auto& indiv : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    individual_graph_->getUpPtr(indiv, up);

    std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

    for(IndivDataRelationElement_t& relation : indiv->data_relations_)
    {
      std::unordered_set<DataPropertyBranch_t*> prop_up;
      individual_graph_->data_property_graph_->getUpPtr(relation.first, prop_up);
      std::unordered_set<ClassBranch_t*> domain;
      for(auto prop : prop_up)
        individual_graph_->data_property_graph_->getDomainPtr(prop, domain);

      if(domain.size() != 0)
      {
        ClassBranch_t* intersection = individual_graph_->class_graph_->firstIntersection(up, domain);
        if(intersection == nullptr)
        {
          std::unordered_set<ClassBranch_t*> disjoints;
          for(auto dom : domain)
            individual_graph_->class_graph_->getDisjoint(dom, disjoints);
          intersection = individual_graph_->class_graph_->firstIntersection(up, disjoints);

          if(intersection == nullptr)
          {
            indiv->flags_["range"].push_back(relation.first->value());
            print_warning("Individual '" + indiv->value() + "' is not in domain of data property '" + relation.first->value() + "'");
          }
          else
            print_error("Individual '" + indiv->value() + "' can not be in domain of data property '" + relation.first->value() + "'");
        }
      }
    }
  }
}

void IndividualChecker::checkDataPropertyRange()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  for(auto& indiv : graph_vect_)
  {
    for(IndivDataRelationElement_t& relation : indiv->data_relations_)
    {
      std::unordered_set<std::string> range = individual_graph_->data_property_graph_->getRange(relation.first->value());
      if(range.size() != 0)
      {
        auto intersection = range.find(relation.second->type_);
        if(intersection == range.end())
          print_error("Individual '" + relation.second->type_ + "' is not in range of '" + relation.first->value() + "'");
      }
    }
  }
}

void IndividualChecker::checkAssymetric()
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  for(auto& indiv : graph_vect_)
  {
    for(IndivObjectRelationElement_t& object_relation : indiv->object_relations_)
    {
      if(object_relation.first->properties_.antisymetric_property_)
        if(symetricExist(indiv, object_relation.first, object_relation.second))
          print_error("'" + object_relation.first->value() + "' is antisymetric so can't be from '" + indiv->value() + "' to '" + object_relation.second->value() + "' and inverse");
    }
  }
}

bool IndividualChecker::symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv)
{
  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);
  for(IndivObjectRelationElement_t& object_relation : sym_indiv->object_relations_)
  {
    if(object_relation.first->get() == sym_prop->get())
      if(object_relation.second->get() == indiv_on->get())
        return true;
  }
  return false;
}

} // namespace ontologenius
