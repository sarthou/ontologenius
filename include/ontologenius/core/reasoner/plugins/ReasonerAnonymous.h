#ifndef ONTOLOGENIUS_REASONERANO_H
#define ONTOLOGENIUS_REASONERANO_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerAnonymous : public ReasonerInterface
{
public:
  ReasonerAnonymous();
  virtual ~ReasonerAnonymous() = default;

  virtual void postReason() override;
  virtual void setParameter(const std::string& name, const std::string& value) override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultActive() override {return true;}
private:
  bool standard_mode_;

  std::unordered_map<ClassBranch_t*, std::unordered_set<ClassBranch_t*>> disjoints_cache_;

  bool checkClassesDisjointess(IndividualBranch_t* indiv, ClassBranch_t* class_equiv);
  int relationExists(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);

  bool resolveFirstLayer(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool resolveTree(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool resolveTree(LiteralNode* literal, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  
  bool checkRestrictionFirstLayer(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem);
  bool checkRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool checkTypeRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool checkTypeRestriction(LiteralNode* literal, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool checkIndividualRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);

  bool checkCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);

  bool checkMinCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool checkMaxCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool checkExactlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool checkOnlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool checkSomeCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
  bool checkValueCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);

  template<typename T>
  bool checkPropertyExistence(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem)
  {
    for(auto& relation : relations)
      if(testBranchInheritanceFirstLayer(ano_elem, relation.first))
        return true;
    return false;
  }

  template<typename T>
  std::vector<std::pair<std::string, size_t>> checkMinCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem , std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::vector<std::pair<std::string, size_t>> indexes;
    std::string explanation;

    for(size_t i = 0; i < relations.size(); i++)
    {
      if(testBranchInheritance(ano_elem, relations[i].first, used))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used))
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            indexes.emplace_back(explanation, i);
            if(indexes.size() >= ano_elem->card_.card_number_)
              return indexes;
          }
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used))
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            indexes.emplace_back(explanation, i);

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
  std::vector<std::pair<std::string, size_t>> checkMaxCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::vector<std::pair<std::string, size_t>> indexes;
    std::string explanation;

    for(size_t i = 0; i < relations.size(); i++)
    {
      if(testBranchInheritance(ano_elem, relations[i].first, used))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used))
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            indexes.emplace_back(explanation, i);

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
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            indexes.emplace_back(explanation, i);

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
  std::vector<std::pair<std::string, size_t>> checkExactlyCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::vector<std::pair<std::string, size_t>> indexes;
    std::string explanation;

    for(size_t i = 0; i < relations.size(); i++)
    {
      if(testBranchInheritance(ano_elem, relations[i].first, used))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used))
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            indexes.emplace_back(explanation, i);
          } 
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used))
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            indexes.emplace_back(explanation, i);
          }
        }
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
  std::vector<std::pair<std::string, size_t>> checkOnlyCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::vector<std::pair<std::string, size_t>> indexes;
    std::string explanation;
    // need to ensure that there is at least one ?
    for(size_t i = 0; i < relations.size(); i++)
    { 
      if(testBranchInheritance(ano_elem, relations[i].first, used))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used) == false)
          {
            used.clear();
            return {};
          }
          else
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            indexes.emplace_back(explanation, i);
          }
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used) == false)
          {
            used.clear();
            return {};
          }
          else
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            indexes.emplace_back(explanation, i);
          }
        }
      }
    }

    return indexes;
  }

  template<typename T>
  std::pair<std::string, int> checkSomeCard(const std::vector<T>& relations, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::string explanation;

    for(size_t i = 0; i < relations.size(); i++)
    {
      if(testBranchInheritance(ano_elem, relations[i].first, used))
      {
        if(ano_elem->is_complex)
        {
          if(resolveTree(relations[i].second, ano_elem->sub_elements_.front(), used) == true)
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            return std::make_pair(explanation, i);
          }
        }
        else
        {
          if(checkTypeRestriction(relations[i].second, ano_elem, used) == true)
          {
            explanation = relations[i].first->value() + "|" + relations[i].second->value() + ";";
            return std::make_pair(explanation, i);
          }  
        }
      }
    }
    return std::make_pair("", -1);
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
  bool testBranchInheritanceFirstLayer(AnonymousClassElement_t* ano_elem, T property)
  {
    auto up_vector = getUpProperty(property);
    for(auto up : up_vector)
      if(testProperty(ano_elem, up))
        return true;
    return false;
  }

  template <typename T>
  bool testBranchInheritance(AnonymousClassElement_t* ano_elem, T* branch, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    if(ano_elem->object_property_involved_ != nullptr)
      return(existInInheritance(branch, ano_elem->object_property_involved_->get(), used));
    else if(ano_elem->data_property_involved_ != nullptr)
      return(existInInheritance(branch, ano_elem->data_property_involved_->get(), used));
    else if(ano_elem->class_involved_ != nullptr)
      return(existInInheritance(branch, ano_elem->class_involved_->get(), used));
    else
      return false;
  }

  template <typename T>
  bool existInInheritance(T* branch, index_t selector, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::string explanation;

    if(branch->get() == selector)
      return true;
    else
    {
      for(size_t i = 0;  i < branch->mothers_.size() ; i++)
      {
        if(existInInheritance(branch->mothers_[i].elem, selector, used))
        {
          explanation = branch->value() + "|isA|" +  branch->mothers_[i].elem->value() + ";";
          used.emplace_back(explanation, branch->mothers_.has_induced_inheritance_relations[i]); 
          return true;
        }  
      } 
    }
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
