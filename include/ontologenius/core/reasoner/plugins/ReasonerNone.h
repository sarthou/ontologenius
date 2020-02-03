#ifndef ONTOLOGENIUS_REASONERNONE_H
#define ONTOLOGENIUS_REASONERNONE_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerNone : public ReasonerInterface
{
public:
  ReasonerNone() {}
  ~ReasonerNone() {}

  virtual void preReason();
  virtual void postReason();
  virtual void periodicReason();

  virtual std::string getName();
  virtual std::string getDesciption();
private:
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERNONE_H
