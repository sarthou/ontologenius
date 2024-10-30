#ifndef ONTOLOGENIUS_PATTERNSSUBSCRIBER_H
#define ONTOLOGENIUS_PATTERNSSUBSCRIBER_H

#include <atomic>
#include <cstddef>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "ontologenius/compat/ros.h"

namespace onto {

  class PatternsSubscriber
  {
  public:
    PatternsSubscriber(const std::string& name = "");
    ~PatternsSubscriber();

    int subscribe(const std::string& pattern, const std::function<void(const std::string&)>& callback, size_t count = -1);
    bool cancel(size_t id);

    bool end() const { return ids_.empty(); }

  private:
    ontologenius::compat::onto_ros::Subscriber<ontologenius::compat::OntologeniusSubscriptionAnswer> sub_;
    ontologenius::compat::onto_ros::Client<ontologenius::compat::OntologeniusSubscription> client_subscribe_;
    ontologenius::compat::onto_ros::Client<ontologenius::compat::OntologeniusUnsubscription> client_cancel_;

    std::atomic<bool> need_to_terminate_;

    std::unordered_map<size_t, std::function<void(const std::string&)>> ids_;

    void patternCallback(const ontologenius::compat::OntologeniusSubscriptionAnswer& msg);

    void spinThread();
  };

} // namespace onto

#endif // ONTOLOGENIUS_PATTERNSSUBSCRIBER_H
