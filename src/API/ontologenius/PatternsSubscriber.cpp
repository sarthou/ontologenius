#include "ontologenius/API/ontologenius/PatternsSubscriber.h"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "ontologenius/compat/ros.h"

namespace onto {

  PatternsSubscriber::PatternsSubscriber(const std::string& name)
    : sub_(name.empty() ? "ontologenius/subscription_answer" : "ontologenius/subscription_answer/" + name, 1000, &PatternsSubscriber::patternCallback, this),
      client_subscribe_(name.empty() ? "ontologenius/subscribe" : "ontologenius/subscribe/" + name),
      client_cancel_(name.empty() ? "ontologenius/unsubscribe" : "ontologenius/unsubscribe/" + name)
  {}

  PatternsSubscriber::~PatternsSubscriber()
  {
    std::vector<size_t> ids;
    ids.reserve(ids_.size());
    for(const auto& id : ids_)
      ids.push_back(id.first);

    for(auto id : ids)
      cancel(id);
  }

  int PatternsSubscriber::subscribe(const std::string& pattern,
                                    const std::function<void(const std::string&)>& callback,
                                    size_t count)
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
      size_t id = ontologenius::compat::onto_ros::getServicePointer(res)->id;
      ids_.emplace(ontologenius::compat::onto_ros::getServicePointer(res)->id, callback);
      return (int)id;
    }
    else
      return -1;
  }

  bool PatternsSubscriber::cancel(size_t id)
  {
    bool done = true;

    auto req = ontologenius::compat::makeRequest<ontologenius::compat::OntologeniusUnsubscription>();
    auto res = ontologenius::compat::makeResponse<ontologenius::compat::OntologeniusUnsubscription>();

    [&](auto&& req) {
      req->id = id;
    }(ontologenius::compat::onto_ros::getServicePointer(req));

    using ResultTy = typename decltype(client_cancel_)::Status_e;

    if(client_cancel_.call(req, res) != ResultTy::ros_status_failure)
    {
      if(ontologenius::compat::onto_ros::getServicePointer(res)->id != (int)id)
        done = false;
    }
    else
      done = false;

    if(done)
      ids_.erase(id);

    return done;
  }

  void PatternsSubscriber::patternCallback(const ontologenius::compat::OntologeniusSubscriptionAnswer& msg)
  {
    auto it = ids_.find(msg.id);
    if(it != ids_.end())
    {
      it->second(msg.data);
      if(msg.last)
        ids_.erase(it);
    }
  }

  void PatternsSubscriber::spinThread()
  {
    while(ontologenius::compat::onto_ros::Node::ok())
      if(need_to_terminate_)
        break;
  }

} // namespace onto
