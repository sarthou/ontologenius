#ifndef ONTOLOGENIUS_SUBSCRIPTION_H
#define ONTOLOGENIUS_SUBSCRIPTION_H

#include <cstddef>
#include <map>
#include <mutex>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"
#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/subscription/IdManager.h"
#include "ontologenius/core/subscription/SubscriptionPattern.h"

namespace ontologenius {

  class Subscription
  {
  public:
    Subscription(Ontology* onto = nullptr) : onto_(onto) {}

    void link(Ontology* onto) { onto_ = onto; }

    size_t subscribe(const SubscriptionPattern& patern, size_t count);
    bool unsubscribe(int id);

    bool isFinished(size_t id);
    bool empty() { return paterns_.empty(); }

    std::vector<size_t> evaluate(const TripletStr_t& triplet);

  private:
    std::map<size_t, SubscriptionPattern> paterns_;
    std::map<size_t, size_t> counts_;
    std::mutex map_mut_;

    IdManager<size_t> id_manager_;

    Ontology* onto_;

    SubscriptionPattern refinePattern(const SubscriptionPattern& triplet);
    bool compareToTriplet(const SubscriptionPattern& pattern, const TripletStr_t& triplet);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_SUBSCRIPTION_H
