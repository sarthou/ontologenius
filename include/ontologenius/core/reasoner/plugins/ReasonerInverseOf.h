#ifndef ONTOLOGENIUS_REASONERINVERSEOF_H
#define ONTOLOGENIUS_REASONERINVERSEOF_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerInverseOf : public ReasonerInterface
{
public:
  ReasonerInverseOf() {}
  virtual ~ReasonerInverseOf() = default;

  virtual void postReason() override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultActive() override {return true;}
private:

  void insertInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* base_prop, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv);
  std::vector<ObjectPropertyElement_t> getLowestInvert(ObjectPropertyBranch_t* base_prop);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERINVERSEOF_H
