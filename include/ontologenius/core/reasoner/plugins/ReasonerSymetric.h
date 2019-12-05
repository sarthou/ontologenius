#ifndef ONTOLOGENIUS_REASONERSYMETRIC_H
#define ONTOLOGENIUS_REASONERSYMETRIC_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerSymetric : public ReasonerInterface
{
public:
  ReasonerSymetric() {}
  ~ReasonerSymetric() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:
  bool symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERSYMETRIC_H
