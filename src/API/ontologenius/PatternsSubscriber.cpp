#include "ontologenius/API/ontologenius/PatternsSubscriber.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <string>

#include "ontologenius/compat/ros.h"

namespace onto {

  PatternsSubscriber::PatternsSubscriber(const std::function<void(const std::string&)>& callback, const std::string& name)
    : sub_(name.empty() ? "ontologenius/subscription_answer" : "ontologenius/subscription_answer/" + name, 1000, &PatternsSubscriber::patternCallback, this),
      client_subscribe_(name.empty() ? "ontologenius/subscribe" : "ontologenius/subscribe/" + name),
      client_cancel_(name.empty() ? "ontologenius/unsubscribe" : "ontologenius/unsubscribe/" + name),
      callback_(callback)
  {}

  PatternsSubscriber::~PatternsSubscriber()
  {
    cancel();
  }

  bool PatternsSubscriber::subscribe(const std::string& pattern, size_t count)
  {
    auto req = ontologenius::compat::makeRequest<ontologenius::compat::OntologeniusSubscription>();
    auto res = ontologenius::compat::makeResponse<ontologenius::compat::OntologeniusSubscription>();

    [&](auto&& req) {
      req->data = pattern;
      req->count = count;
    }(ontologenius::compat::onto_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_subscribe_)::Status_e;

    if(client_subscribe_.call(req, res) != ResultTy::ros_status_failure)
    {
      ids_.push_back(ontologenius::compat::onto_ros::getServicePointer(res)->id);
      return true;
    }
    else
      return false;
  }

  bool PatternsSubscriber::cancel()
  {
    bool done = true;
    for(size_t i = 0; i < ids_.size();)
    {
      auto req = ontologenius::compat::makeRequest<ontologenius::compat::OntologeniusUnsubscription>();
      auto res = ontologenius::compat::makeResponse<ontologenius::compat::OntologeniusUnsubscription>();

      [&](auto&& req) {
        req->id = ids_[i];
      }(ontologenius::compat::onto_ros::getServicePointer(req));

      using ResultTy = typename decltype(client_cancel_)::Status_e;

      if(client_cancel_.call(req, res) != ResultTy::ros_status_failure)
      {
        if(ontologenius::compat::onto_ros::getServicePointer(res)->id != (int)ids_[i])
        {
          done = false;
        }
      }
      else
      {
        done = false;
      }

      if(done)
        ids_.erase(ids_.begin() + (int)i);
      else
        i++;
    }

    return done;
  }

  void PatternsSubscriber::patternCallback(const ontologenius::compat::OntologeniusSubscriptionAnswer& msg)
  {
    auto it = std::find(ids_.begin(), ids_.end(), msg.id);
    if(it != ids_.end())
    {
      callback_(msg.data);
      if(msg.last)
        ids_.erase(it);
    }
  }

  void PatternsSubscriber::spinThread()
  {
    while(ontologenius::compat::onto_ros::Node::ok())
    {
      if(need_to_terminate_)
      {
        break;
      }
    }
  }

} // namespace onto
