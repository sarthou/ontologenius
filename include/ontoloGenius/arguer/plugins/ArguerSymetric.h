#ifndef ARGUERSYMETRIC_H
#define ARGUERSYMETRIC_H

#include "ontoloGenius/arguer/plugins/ArguerInterface.h"

class ArguerSymetric : public ArguerInterface
{
public:
  ArguerSymetric() {}
  ~ArguerSymetric() {}

  void preReason();
  void postReason();

  std::string getName();
  std::string getDesciption();

  bool defaultAvtive() {return true;}
private:
  bool symetricExist(IndividualBranch_t* indiv_on, PropertyClassBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
};

#endif
