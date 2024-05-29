#ifndef ONTOLOGENIUS_REASONERNONE_H
#define ONTOLOGENIUS_REASONERNONE_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  class ReasonerNone : public ReasonerInterface
  {
  public:
    ReasonerNone() = default;
    ~ReasonerNone() override = default;

    bool preReason(const QueryInfo_t& query_info) override;
    void postReason() override;
    bool periodicReason() override;

    bool implementPreReasoning() override { return true; }
    bool implementPostReasoning() override { return true; }
    bool implementPeriodicReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERNONE_H
