#ifndef ONTOLOGENIUS_REASONERINVERSEOF_H
#define ONTOLOGENIUS_REASONERINVERSEOF_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  class ReasonerInverseOf : public ReasonerInterface
  {
  public:
    ReasonerInverseOf() = default;
    ~ReasonerInverseOf() override = default;

    void postReason() override;

    bool implementPostReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;

    bool defaultActive() override { return true; }

  private:
    void insertInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* base_prop, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv);
    std::vector<ObjectPropertyElement_t> getLowestInvert(ObjectPropertyBranch_t* base_prop);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERINVERSEOF_H
