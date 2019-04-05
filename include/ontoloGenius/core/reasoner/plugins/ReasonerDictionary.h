#ifndef ONTOLOGENIUS_REASONERDICTIONARY_H
#define ONTOLOGENIUS_REASONERDICTIONARY_H

#include "ontoloGenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerDictionary : public ReasonerInterface
{
public:
  ReasonerDictionary() {}
  ~ReasonerDictionary() {}

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

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERDICTIONARY_H
