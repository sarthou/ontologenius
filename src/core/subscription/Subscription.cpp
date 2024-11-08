#include "ontologenius/core/subscription/Subscription.h"

#include <cstddef>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"
#include "ontologenius/core/subscription/SubscriptionPattern.h"

namespace ontologenius {

  size_t Subscription::subscribe(const SubscriptionPattern& patern, size_t count)
  {
    map_mut_.lock();
    size_t id = id_manager_.getNewId();
    paterns_.insert(std::pair<size_t, SubscriptionPattern>(id, refinePattern(patern)));
    counts_[id] = count;
    map_mut_.unlock();

    return id;
  }

  bool Subscription::unsubscribe(int id)
  {
    bool res = true;
    map_mut_.lock();
    if(id != -1)
    {
      if(id_manager_.removeId(id))
      {
        paterns_.erase(id);
        counts_.erase(id);
      }
      else
        res = false;
    }
    else
    {
      auto ids = id_manager_.getIds();
      for(auto id_to_remove : ids)
      {
        if(id_manager_.removeId(id_to_remove))
        {
          paterns_.erase(id_to_remove);
          counts_.erase(id_to_remove);
        }
        else
          res = false;
      }
    }

    map_mut_.unlock();
    return res;
  }

  bool Subscription::isFinished(size_t id)
  {
    bool res = true;

    map_mut_.lock();
    if(counts_.find(id) != counts_.end())
      res = (counts_[id] == 0);
    map_mut_.unlock();

    return res;
  }

  std::vector<size_t> Subscription::evaluate(const TripletStr_t& triplet)
  {
    std::vector<size_t> res;

    map_mut_.lock();
    for(auto& it : paterns_)
    {
      if(compareToTriplet(it.second, triplet))
      {
        if(counts_[it.first] != 0)
        {
          res.push_back(it.first);
          counts_[it.first]--;
        }
      }
    }
    map_mut_.unlock();

    return res;
  }

  SubscriptionPattern Subscription::refinePattern(const SubscriptionPattern& triplet)
  {
    SubscriptionPattern pattern = triplet;
    if(onto_ != nullptr)
    {
      if(pattern.isSubjectUndefined() == false)
        if(onto_->class_graph_.getUp(triplet.subject()).empty() == false)
          pattern.setSubjectAsClass();
      if(pattern.isObjectUndefined() == false)
        if(onto_->class_graph_.getUp(triplet.object()).empty() == false)
          pattern.setObjectAsClass();
      if(pattern.isPredicatUndefined() == false)
        if(onto_->data_property_graph_.getUp(triplet.predicate()).empty() == false)
          pattern.setPredicatAsDataProperty();
    }

    return pattern;
  }

  bool Subscription::compareToTriplet(const SubscriptionPattern& pattern, const TripletStr_t& triplet)
  {
    if(onto_ == nullptr)
      return pattern.fit(triplet);

    if(!pattern.isOperatorUndefined())
      if(pattern.add() != triplet.add)
        return false;

    if(pattern.isSubjectIndividual() && !pattern.isSubjectUndefined())
    {
      if(pattern.subject() != triplet.subject)
        return false;
    }

    if(pattern.isObjectIndividual() && !pattern.isObjectUndefined())
    {
      if(pattern.object() != triplet.object)
        return false;
    }

    if(pattern.isSubjectIndividual() == false)
    {
      if(onto_->individual_graph_.isA(triplet.subject, pattern.subject()) == false)
        return false;
    }
    // subject match

    if(pattern.isObjectIndividual() == false)
    {
      if((onto_->individual_graph_.isA(triplet.object, pattern.object()) == false) &&
         (onto_->class_graph_.isA(triplet.object, pattern.object()) == false))
        return false;
    }
    // object match

    if((pattern.predicate() != triplet.predicate) && !pattern.isPredicatUndefined())
    {
      if(pattern.isPredicatObjectProperty())
      {
        if(onto_->object_property_graph_.isA(triplet.predicate, pattern.predicate()) == false)
          return false;
      }
      else
      {
        if(onto_->data_property_graph_.isA(triplet.predicate, pattern.predicate()) == false)
          return false;
      }
    }
    // predicat match

    return true;
  }

} // namespace ontologenius
