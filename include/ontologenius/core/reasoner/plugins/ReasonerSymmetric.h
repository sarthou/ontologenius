#ifndef ONTOLOGENIUS_REASONERSYMMETRIC_H
#define ONTOLOGENIUS_REASONERSYMMETRIC_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerSymmetric : public ReasonerInterface
{
public:
  ReasonerSymmetric() {}
  virtual ~ReasonerSymmetric() = default;

  virtual void postReason() override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultActive() override {return true;}
private:
  bool symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERSYMMETRIC_H
