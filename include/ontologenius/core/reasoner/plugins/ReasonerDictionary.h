#ifndef ONTOLOGENIUS_REASONERDICTIONARY_H
#define ONTOLOGENIUS_REASONERDICTIONARY_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerDictionary : public ReasonerInterface
{
public:
  ReasonerDictionary() { use_id_ = false; }
  virtual ~ReasonerDictionary() = default;

  void setParameter(const std::string& name, const std::string& value);

  virtual void postReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:
  bool use_id_;

  void updateDictionary(ValuedNode* node);

  void setId(ValuedNode* node);
  void split(ValuedNode* node);
  void createLowerCase(ValuedNode* node);
  void replaceQuote(ValuedNode* node);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERDICTIONARY_H
