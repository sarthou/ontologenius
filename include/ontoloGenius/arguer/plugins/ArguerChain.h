#ifndef ARGUERCHAIN_H
#define ARGUERCHAIN_H

#include "ontoloGenius/arguer/plugins/ArguerInterface.h"

class ArguerChain : public ArguerInterface
{
public:
  ArguerChain() {}
  ~ArguerChain() {}

  void preReason();
  void postReason();

  std::string getName();
  std::string getDesciption();

  bool defaultAvtive() {return true;}
private:
  void resolveChain(std::vector<PropertyClassBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on);
  void resolveLink(PropertyClassBranch_t* chain_property, std::vector<IndividualBranch_t*>& indivs);
};

#endif
