#ifndef ARGUERDICTIONARY_H
#define ARGUERDICTIONARY_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

class ArguerDictionary : public ArguerInterface
{
public:
  ArguerDictionary() {}
  ~ArguerDictionary() {}

  virtual void preReason();
  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:
  void updateDictionary(ValuedNode* node);

  void split(ValuedNode* node);
  void createLowerCase(ValuedNode* node);
  void replaceQuote(ValuedNode* node);
};

#endif
