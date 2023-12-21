#ifndef ONTOLOGENIUS_REASONERANO_H
#define ONTOLOGENIUS_REASONERANO_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

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
  std::string computeExplanation( std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkClassesDisjointess(IndividualBranch_t* indiv, ClassBranch_t* class_equiv);
  int relationExists(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);

  bool resolveFirstLayer(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool resolveTree(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool resolveTree(LiteralNode* literal, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  
  bool checkRestrictionFirstLayer(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkTypeRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkTypeRestriction(LiteralNode* literal, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkIndividualRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);

  bool checkCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);

  bool checkMinCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkMaxCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkExactlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkOnlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkSomeCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);
  bool checkValueCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used);

  template<typename T>
  bool checkPropertyExistence(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem)
  {
    for(auto& relation : relations)
      if(testPropertyInheritance(ano_elem, relation.first))
        return true;
    return false;
  }

  template<typename T>
  std::vector<size_t> checkMinCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem , std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used)
  {
    std::vector<size_t> indexes;

    for(size_t i = 0; i < relations.size(); i++)
    {
      if(testPropertyInheritance(ano_elem, relations[i].first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used))
          {
            indexes.push_back(i);
            if(indexes.size() >= ano_elem->card_.card_number_)
              return indexes;
          }
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used))
          {
            indexes.push_back(i);
            if(indexes.size() >= ano_elem->card_.card_number_)
              return indexes;
          }
        }
      }
    }
    used.clear();
    return {};
  }

  template<typename T>
  std::vector<size_t> checkMaxCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used)
  {
    std::vector<size_t> indexes;

    for(size_t i = 0; i < relations.size(); i++)
    {
      if(testPropertyInheritance(ano_elem, relations[i].first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used))
          {
            indexes.push_back(i);
            if(indexes.size() > ano_elem->card_.card_number_)
            {
              used.clear();
              return {};
            }
          }
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used))
          {
            indexes.push_back(i);
            if(indexes.size() > ano_elem->card_.card_number_)
            {
              used.clear();
              return {};
            }
          }
        }
      }
    }
    return indexes;
  }

  template<typename T>
  std::vector<size_t> checkExactlyCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used)
  {
    std::vector<size_t> indexes;

    for(size_t i = 0; i < relations.size(); i++)
      if(testPropertyInheritance(ano_elem, relations[i].first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used))
            indexes.push_back(i);
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used))
            indexes.push_back(i);
        }
      }

    if(indexes.size() == ano_elem->card_.card_number_)
      return indexes;
    else
    {
      used.clear();
      return {};
    }
  }

  template<typename T>
  std::vector<size_t> checkOnlyCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used)
  {
    std::vector<size_t> indexes;

    // need to ensure that there is at least one ?
    for(size_t i = 0; i < relations.size(); i++)
    { 
      if(testPropertyInheritance(ano_elem, relations[i].first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used) == false)
          {
            used.clear();
            return {};
          }
          else
            indexes.push_back(i);
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used) == false)
          {
            used.clear();
            return {};
          }
          else
            indexes.push_back(i);
        }
      }
    }
    return indexes;
  }

  template<typename T>
  int checkSomeCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem,  std::vector<std::pair< ProbabilisticElement_t*, InheritedRelationTriplets*>>& used)
  {
    for(size_t i = 0; i < relations.size(); i++)
    {
      if(testPropertyInheritance(ano_elem, relations[i].first))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used) == true)
            return i;
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used) == true)
            return i;
        }
      }
    }
    return -1;
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
