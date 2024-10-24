#include "ontologenius/core/subscription/SubscriptionManager.h"

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include "ontologenius/compat/ros.h"
#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"
#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/subscription/SubscriptionPattern.h"

namespace ontologenius {

  SubscriptionManager::SubscriptionManager(const std::string& name) : run_(false),
                                                                      pub_((name.empty()) ? "ontologenius/subscription_answer" : "ontologenius/subscription_answer/" + name, 1000),
                                                                      sub_service_(name.empty() ? "ontologenius/subscribe" : "ontologenius/subscribe/" + name, &SubscriptionManager::subscribeCallback, this),
                                                                      unsub_service_(name.empty() ? "ontologenius/unsubscribe" : "ontologenius/unsubscribe/" + name, &SubscriptionManager::unsubscribeCallback, this)
  {
  }

  void SubscriptionManager::run()
  {
    run_ = true;
    compat::onto_ros::Rate r(50);

    while(compat::onto_ros::Node::ok() && isRunning())
    {
      while(empty() == false)
      {
        TripletStr_t triplet = get();
        if(triplet.valid())
        {
          std::vector<size_t> ids = subscription_.evaluate(triplet);
          for(auto id : ids)
          {
            compat::OntologeniusSubscriptionAnswer msg;
            msg.id = (int)id;
            msg.data = triplet.toString();
            msg.last = subscription_.isFinished(id);
            if(msg.last)
              subscription_.unsubscribe((int)id);
            pub_.publish(msg);
          }
        }
      }
      r.sleep();
    }
  }

  void SubscriptionManager::add(const TripletStr_t& triplet)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    triplets_.push(triplet);
  }

  void SubscriptionManager::add(const std::string& triplet_str)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto pattern(SubscriptionPattern::deserialize(triplet_str));
    triplets_.push(pattern.getTriplet());
  }

  void SubscriptionManager::add(const std::vector<std::pair<std::string, std::string>>& explanations)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for(auto& expl : explanations)
    {
      auto pattern(SubscriptionPattern::deserialize(expl.first));
      triplets_.push(pattern.getTriplet());
    }
  }

  bool SubscriptionManager::subscribeCallback(compat::onto_ros::ServiceWrapper<compat::OntologeniusSubscription::Request>& req,
                                              compat::onto_ros::ServiceWrapper<compat::OntologeniusSubscription::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      auto pattern = SubscriptionPattern::deserialize(req->data);

      if(pattern.valid() == false)
        return false;

      res->id = subscription_.subscribe(pattern, req->count);

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool SubscriptionManager::unsubscribeCallback(compat::onto_ros::ServiceWrapper<compat::OntologeniusUnsubscription::Request>& req,
                                                compat::onto_ros::ServiceWrapper<compat::OntologeniusUnsubscription::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      if(subscription_.unsubscribe(req->id))
        res->id = req->id;
      else
        res->id = -1;

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  TripletStr_t SubscriptionManager::get()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TripletStr_t res(triplets_.front());
    triplets_.pop();
    return res;
  }

  bool SubscriptionManager::empty()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return triplets_.empty();
  }

} // namespace ontologenius
