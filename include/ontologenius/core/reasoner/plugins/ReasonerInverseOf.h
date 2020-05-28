#ifndef ONTOLOGENIUS_REASONERINVERSEOF_H
#define ONTOLOGENIUS_REASONERINVERSEOF_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerInverseOf : public ReasonerInterface
{
public:
  ReasonerInverseOf() {}
  ~ReasonerInverseOf() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:

  void insertInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv);
  std::vector<ObjectPropertyElement_t> getLowestInvert(ObjectPropertyBranch_t* base_prop);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERINVERSEOF_H
