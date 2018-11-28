#ifndef ARGUERGENERALIZE_H
#define ARGUERGENERALIZE_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

class ArguerGeneralize : public ArguerInterface
{
public:
  ArguerGeneralize() {}
  ~ArguerGeneralize() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:
};

#endif
