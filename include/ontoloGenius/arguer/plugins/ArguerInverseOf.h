#ifndef ARGUERINVERSEOF_H
#define ARGUERINVERSEOF_H

#include "ontoloGenius/arguer/plugins/ArguerInterface.h"

class ArguerInverseOf : public ArguerInterface
{
public:
  ArguerInverseOf() {}
  ~ArguerInverseOf() {}

  void preReason();
  void postReason();

  std::string getName();
  std::string getDesciption();

  bool defaultAvtive() {return true;}
private:

  void insetInverse(IndividualBranch_t* indiv_on, PropertyClassBranch_t* inv_prop, IndividualBranch_t* inv_indiv);
};

#endif
