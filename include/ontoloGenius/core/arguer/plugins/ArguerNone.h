#ifndef ARGUERNONE_H
#define ARGUERNONE_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

class ArguerNone : public ArguerInterface
{
public:
  ArguerNone() {}
  ~ArguerNone() {}

  void preReason();
  void postReason();

  std::string getName();
  std::string getDesciption();
private:
};

#endif
