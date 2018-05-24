#ifndef ARGUERNONE_H
#define ARGUERNONE_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

class ArguerNone : public ArguerInterface
{
public:
  ArguerNone() {}
  ~ArguerNone() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();
private:
};

#endif
