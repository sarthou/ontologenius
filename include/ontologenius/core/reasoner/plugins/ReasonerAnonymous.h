#ifndef ONTOLOGENIUS_REASONERANO_H
#define ONTOLOGENIUS_REASONERANO_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ChainTree;
class ReasonerAno : public ReasonerInterface
{
public:
  ReasonerAno() {}
  ~ReasonerAno() {}

  virtual void postReason() override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultAvtive() override {return true;}
private:

  bool relationExists(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* property, IndividualBranch_t* chain_indiv);

  bool resolveTree(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool resolveTree(LiteralNode* literal, AnonymousClassElement_t* ano_elem);
  
  bool checkRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkClassRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkIndividualRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkLiteralRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  DataPropertyBranch_t* data_property);

  bool checkCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);

  bool checkMinCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkMaxCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkExactlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkOnlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkSomeCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkValueCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);

  template<typename T>
  bool checkMinCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem)
  {
    size_t nb_card = 0;
    for(auto& relation : relations)
    {
      if(resolveTree(relation.second, ano_elem->sub_elements_.front()))
      {
        nb_card++;
        if(nb_card >= ano_elem->card_.card_number_)
          return true;
      }
    }
    return false;
  }

};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERANO_H
