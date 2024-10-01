#ifndef ONTOLOGENIUS_REASONERRULE_H
#define ONTOLOGENIUS_REASONERRULE_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerRule : public ReasonerInterface
{
public:
  ReasonerRule() {}
  ~ReasonerRule() {}

  virtual void postReason() override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultActive() override {return true;}
private:

};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERRULE_H