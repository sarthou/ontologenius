#ifndef ARGUERSYMETRIC_H
#define ARGUERSYMETRIC_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

class ArguerSymetric : public ArguerInterface
{
public:
  ArguerSymetric() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:
  bool symetricExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
};

#endif
