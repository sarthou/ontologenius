#ifndef ONTOLOGENIUS_SUBSCRIPTIONMANAGER_H
#define ONTOLOGENIUS_SUBSCRIPTIONMANAGER_H

#include <atomic>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "ontologenius/compat/ros.h"
#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"
#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/subscription/Subscription.h"

namespace ontologenius {

  class SubscriptionManager
  {
  public:
    explicit SubscriptionManager(const std::string& name = "");

    void link(Ontology* onto) { subscription_.link(onto); }

    void run();

    void add(const TripletStr_t& triplet);
    void add(const std::string& triplet_str);
    void add(const std::vector<std::pair<std::string, std::string>>& explanations);

    void stop() { run_ = false; }
    bool isRunning() const { return run_; }

  private:
    Subscription subscription_;
    std::atomic<bool> run_;

    compat::onto_ros::Publisher<compat::OntologeniusSubscriptionAnswer> pub_;
    compat::onto_ros::Service<compat::OntologeniusSubscription> sub_service_;
    compat::onto_ros::Service<compat::OntologeniusUnsubscription> unsub_service_;

    std::mutex mutex_;
    std::queue<TripletStr_t> triplets_;

    bool subscribeCallback(compat::onto_ros::ServiceWrapper<compat::OntologeniusSubscription::Request>& req,
                           compat::onto_ros::ServiceWrapper<compat::OntologeniusSubscription::Response>& res);

    bool unsubscribeCallback(compat::onto_ros::ServiceWrapper<compat::OntologeniusUnsubscription::Request>& req,
                             compat::onto_ros::ServiceWrapper<compat::OntologeniusUnsubscription::Response>& res);

    TripletStr_t get();
    bool empty();
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_SUBSCRIPTIONMANAGER_H
