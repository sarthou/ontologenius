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

  virtual std::string getName() override;
  virtual std::string getDesciption() override;

  virtual bool defaultAvtive() override {return true;}
private:

  void postReasonIndividuals();
  void deduceRange(IndividualBranch_t* branch, std::string& prop);
  void deduceDomain(IndividualBranch_t* branch, std::string& prop);

  void deduceObjRange(IndividualBranch_t* branch, size_t index);
  void deduceDatRange(IndividualBranch_t* branch, size_t index);
  void deduceObjDomain(IndividualBranch_t* branch, size_t index);
  void deduceDatDomain(IndividualBranch_t* branch, size_t index);


  void postReasonClasses();
  void deduceRange(ClassBranch_t* branch, std::string& prop);
  void deduceDomain(ClassBranch_t* branch, std::string& prop);

  void deduceObjRange(ClassBranch_t* branch, size_t index);
  void deduceDatRange(ClassBranch_t* branch, size_t index);
  void deduceObjDomain(ClassBranch_t* branch, size_t index);
  void deduceDatDomain(ClassBranch_t* branch, size_t index);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERRANGEDOMAIN_H
