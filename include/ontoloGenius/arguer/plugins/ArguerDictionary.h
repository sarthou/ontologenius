#ifndef ARGUERDICTIONARY_H
#define ARGUERDICTIONARY_H

#include "ontoloGenius/arguer/plugins/ArguerInterface.h"

class ArguerDictionary : public ArguerInterface
{
public:
  ArguerDictionary() {}
  ~ArguerDictionary() {}

  void preReason();
  void postReason();

  std::string getName();
  std::string getDesciption();

  bool defaultAvtive() {return true;}
private:

  void split(IndividualBranch_t* indiv);
  void createLowerCase(IndividualBranch_t* indiv);
  void replaceQuote(IndividualBranch_t* indiv);
};

#endif
