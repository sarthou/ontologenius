#ifndef ONTOLOGENIUS_REASONERRANGEDOMAIN_H
#define ONTOLOGENIUS_REASONERRANGEDOMAIN_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  class ReasonerRangeDomain : public ReasonerInterface
  {
  public:
    ReasonerRangeDomain() = default;
    ~ReasonerRangeDomain() override = default;

    void postReason() override;

    bool implementPostReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;

    bool defaultActive() override { return true; }

  private:
    void postReasonIndividuals();
    void deduceRange(IndividualBranch* branch, const std::string& prop);
    void deduceDomain(IndividualBranch* branch, const std::string& prop);

    void deduceObjRange(IndivObjectRelationElement& relation);
    void deduceObjDomain(IndividualBranch* branch, IndivObjectRelationElement& relation);
    void deduceDatDomain(IndividualBranch* branch, IndivDataRelationElement& relation);

    void postReasonClasses();
    void deduceRange(ClassBranch* branch, const std::string& prop);
    void deduceDomain(ClassBranch* branch, const std::string& prop);

    void deduceObjRange(ClassObjectRelationElement_t& relation);
    void deduceObjDomain(ClassBranch* branch, ClassObjectRelationElement_t& relation);
    void deduceDatDomain(ClassBranch* branch, ClassDataRelationElement_t& relation);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERRANGEDOMAIN_H
