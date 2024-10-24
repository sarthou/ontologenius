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
    PatternsSubscriber(const std::function<void(const std::string&)>& callback, const std::string& name = "");
    ~PatternsSubscriber();

    bool subscribe(const std::string& pattern, size_t count = -1);
    bool cancel();

    bool end() const { return ids_.empty(); }

  private:
    ontologenius::compat::onto_ros::Subscriber<ontologenius::compat::OntologeniusSubscriptionAnswer> sub_;
    ontologenius::compat::onto_ros::Client<ontologenius::compat::OntologeniusSubscription> client_subscribe_;
    ontologenius::compat::onto_ros::Client<ontologenius::compat::OntologeniusUnsubscription> client_cancel_;

    std::atomic<bool> need_to_terminate_;

    std::vector<size_t> ids_;

    void patternCallback(const ontologenius::compat::OntologeniusSubscriptionAnswer& msg);

    std::function<void(const std::string&)> callback_;

    void spinThread();
  };

} // namespace onto

#endif // ONTOLOGENIUS_PATTERNSSUBSCRIBER_H
