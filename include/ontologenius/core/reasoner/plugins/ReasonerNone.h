#ifndef ONTOLOGENIUS_REASONERNONE_H
#define ONTOLOGENIUS_REASONERNONE_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerNone : public ReasonerInterface
{
public:
  ReasonerNone() {}
  virtual ~ReasonerNone() = default;

  virtual void preReason(const QueryInfo_t& query_info) override;
  virtual void postReason() override;
  virtual void periodicReason() override;

  virtual std::string getName() override;
  virtual std::string getDesciption() override;
private:
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERNONE_H
