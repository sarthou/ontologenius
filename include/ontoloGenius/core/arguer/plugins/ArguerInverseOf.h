#ifndef ARGUERINVERSEOF_H
#define ARGUERINVERSEOF_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

class ArguerInverseOf : public ArguerInterface
{
public:
  ArguerInverseOf() {}
  ~ArguerInverseOf() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:

  void insetInverse(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* inv_prop, IndividualBranch_t* inv_indiv);
};

#endif
