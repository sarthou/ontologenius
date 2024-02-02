#ifndef ONTOLOGENIUS_REASONERRANGEDOMAIN_H
#define ONTOLOGENIUS_REASONERRANGEDOMAIN_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerRangeDomain : public ReasonerInterface
{
public:
  ReasonerRangeDomain() {}
  virtual ~ReasonerRangeDomain() = default;

  virtual void postReason() override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultActive() override {return true;}
private:

  void postReasonIndividuals();
  void deduceRange(IndividualBranch_t* branch, const std::string& prop);
  void deduceDomain(IndividualBranch_t* branch, const std::string& prop);

  void deduceObjRange(IndivObjectRelationElement_t& relation);
  void deduceObjDomain(IndividualBranch_t* branch, IndivObjectRelationElement_t& relation);
  void deduceDatDomain(IndividualBranch_t* branch, IndivDataRelationElement_t& relation);


  void postReasonClasses();
  void deduceRange(ClassBranch_t* branch, const std::string& prop);
  void deduceDomain(ClassBranch_t* branch, const std::string& prop);

  void deduceObjRange(ClassObjectRelationElement_t& relation);
  void deduceObjDomain(ClassBranch_t* branch, ClassObjectRelationElement_t& relation);
  void deduceDatDomain(ClassBranch_t* branch, ClassDataRelationElement_t& relation);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERRANGEDOMAIN_H
