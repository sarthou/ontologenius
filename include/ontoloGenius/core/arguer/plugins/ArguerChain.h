#ifndef ARGUERCHAIN_H
#define ARGUERCHAIN_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

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
  void resolveChain(std::vector<ObjectPropertyBranch_t*> chain, IndividualBranch_t* indiv, IndividualBranch_t* on);
  void resolveLink(ObjectPropertyBranch_t* chain_property, std::vector<IndividualBranch_t*>& indivs);
  bool porpertyExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* chain_prop, IndividualBranch_t* chain_indiv);
};

#endif
