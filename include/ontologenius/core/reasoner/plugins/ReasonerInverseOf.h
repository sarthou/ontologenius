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
    void insertInverse(IndividualBranch* indiv_on, ObjectPropertyBranch* base_prop, ObjectPropertyBranch* inv_prop, IndividualBranch* inv_indiv);
    std::vector<ObjectPropertyElement> getLowestInvert(ObjectPropertyBranch* base_prop);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERINVERSEOF_H
