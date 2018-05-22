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
  void updateDictionary(ValuedNode* node);

  void split(ValuedNode* node);
  void createLowerCase(ValuedNode* node);
  void replaceQuote(ValuedNode* node);
};

#endif
