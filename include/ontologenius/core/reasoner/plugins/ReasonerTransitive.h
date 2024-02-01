#ifndef ONTOLOGENIUS_REASONERTRANSITIVE_H
#define ONTOLOGENIUS_REASONERTRANSITIVE_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerTransitive : public ReasonerInterface
{
public:
  ReasonerTransitive() {}
  virtual ~ReasonerTransitive() = default;
  
  virtual void postReason() override;
  
  virtual bool implementPostReasoning() override { return true; }

  bool transitiveExist(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* sym_prop, IndividualBranch_t* sym_indiv);
  virtual std::string getName() override;
  virtual std::string getDescription() override;
private:

};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERTRANSITIVE_H
