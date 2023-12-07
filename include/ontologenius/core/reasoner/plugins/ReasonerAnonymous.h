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

  virtual bool defaultActive() override {return true;}
private:

  bool relationExists(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);

  bool resolveTree(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool resolveTree(LiteralNode* literal, AnonymousClassElement_t* ano_elem);
  
  bool checkRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkTypeRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkTypeRestriction(LiteralNode* literal, AnonymousClassElement_t* ano_elem);
  bool checkIndividualRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);

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
      if(testPropertyInheritance(ano_elem, relation.first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relation.second, ano_elem->sub_elements_.front()))
          {
            nb_card++;
            if(nb_card >= ano_elem->card_.card_number_)
              return true;
          }
        }
        else
        {
          if(resolveTree(relation.second, ano_elem))
          {
            nb_card++;
            if(nb_card >= ano_elem->card_.card_number_)
              return true;
          }
        }
      }
    }
    return false;
  }

  template<typename T>
  bool checkMaxCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem)
  {
    size_t nb_card = 0;

    for(auto& relation : relations)
    {
      if(testPropertyInheritance(ano_elem, relation.first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relation.second, ano_elem->sub_elements_.front()))
          {
            nb_card++;
            if(nb_card > ano_elem->card_.card_number_)
              return false;
          }
        }
        else
        {
          if(resolveTree(relation.second, ano_elem))
          {
            nb_card++;
            if(nb_card > ano_elem->card_.card_number_)
              return false;
          }
        }
      }
    }
    return true;
  }

  template<typename T>
  bool checkExactlyCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem)
  {
    size_t nb_card = 0;

    for(auto& relation : relations)
      if(testPropertyInheritance(ano_elem, relation.first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relation.second, ano_elem->sub_elements_.front()))
             nb_card++;
        }
        else
        {
          if(checkTypeRestriction(relation.second, ano_elem))
             nb_card++;
        }
      }

    if(nb_card == ano_elem->card_.card_number_)
      return true;
    else
      return false;
  }

  template<typename T>
  bool checkOnlyCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem)
  {
    // need to ensure that there is at least one ?
    for(auto& relation : relations)
    { 
      if(testPropertyInheritance(ano_elem, relation.first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relation.second, ano_elem->sub_elements_.front()) == false)
            return false;
        }
        else
        {
          if(checkTypeRestriction(relation.second, ano_elem) == false)
            return false;
        }
      }
    }
    return true;
  }

  template<typename T>
  bool checkSomeCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem)
  {
    for(auto& relation : relations)
      if(testPropertyInheritance(ano_elem, relation.first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relation.second, ano_elem->sub_elements_.front()))
            return true;
        }
        else
        {
          if(checkTypeRestriction(relation.second, ano_elem))
            return true;
        }
      }
    return false;
  }

  inline std::unordered_set<ObjectPropertyBranch_t*> getUpProperty(ObjectPropertyBranch_t* property)
  {
    return ontology_->object_property_graph_.getUpPtrSafe(property);
  }

  inline std::unordered_set<DataPropertyBranch_t*> getUpProperty(DataPropertyBranch_t* property)
  {
    return ontology_->data_property_graph_.getUpPtrSafe(property);
  }

  template<typename T>
  bool testPropertyInheritance(AnonymousClassElement_t* ano_elem, T property)
  {
    auto up_vector = getUpProperty(property);
    for(auto property : up_vector)
      if(testProperty(ano_elem, property))
        return true;
    return false;
  }

  inline bool testProperty(AnonymousClassElement_t* ano_elem, ObjectPropertyBranch_t* property)
  {
    return ano_elem->object_property_involved_ == property;
  }

  inline bool testProperty(AnonymousClassElement_t* ano_elem, DataPropertyBranch_t* property)
  {
    return ano_elem->data_property_involved_ == property;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERANO_H
