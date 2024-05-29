#ifndef ONTOLOGENIUS_REASONERSYMMETRIC_H
#define ONTOLOGENIUS_REASONERSYMMETRIC_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  class ReasonerSymmetric : public ReasonerInterface
  {
  public:
    ReasonerSymmetric() = default;
    ~ReasonerSymmetric() override = default;

    void postReason() override;

    bool implementPostReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;

    bool defaultActive() override { return true; }

  private:
    bool symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERSYMMETRIC_H
