#ifndef REASONERSYMETRIC_H
#define REASONERSYMETRIC_H

#include "ontoloGenius/core/reasoner/plugins/ReasonerInterface.h"

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

#endif
