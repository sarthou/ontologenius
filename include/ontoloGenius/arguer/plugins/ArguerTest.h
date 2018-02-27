#ifndef ARGUERTEST_H
#define ARGUERTEST_H

#include "ontoloGenius/arguer/plugins/ArguerInterface.h"

class ArguerTest : public ArguerInterface
{
public:
  ArguerTest() {}
  ~ArguerTest() {}

  void preReason();
  void postReason();

  std::string getName();
  std::string getDesciption();
private:
};

#endif
