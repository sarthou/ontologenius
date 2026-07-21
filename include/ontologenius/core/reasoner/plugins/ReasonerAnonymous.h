#ifndef ONTOLOGENIUS_REASONERANO_H
#define ONTOLOGENIUS_REASONERANO_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  class ReasonerAnonymous : public ReasonerInterface
  {
  public:
    ReasonerAnonymous();
    ~ReasonerAnonymous() override = default;

    void postReason() override;
    void setParameter(const std::string& name, const std::string& value) override;

    bool implementPostReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;

    bool defaultActive() override { return true; }

  private:
    bool standard_mode_;
    IndividualBranch* current_individual_;
    bool has_involved_other_individual_;

    std::unordered_map<ClassBranch*, std::unordered_set<ClassBranch*>> disjoints_cache_;

    bool resolveTree(LiteralNode* literal, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
    bool resolveIdentifier(LiteralNode* literal, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);          // Type 1
    bool resolveOneOfDatatype(LiteralNode* literal, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);       // Type 2

    bool resolveTree(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
    bool resolveIdentifier(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);       // Type 1
    bool resolveOneOfIndividual(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);  // Type 2
    bool resolveRestriction(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);      // Type 3

    bool compareIndividuals(IndividualBranch* indiv, IndividualBranch* other, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
    bool checkValue(IndividualBranch* indiv_from, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
    bool checkValue(LiteralNode* literal_from, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);

    bool resolveAllValuesFrom(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
    bool resolveSomeValuesFrom(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
    bool resolveHasValue(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
    bool resolveMaxCardinality(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);
    bool resolveMinCardinality(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);

    bool checkClassesDisjointess(IndividualBranch* indiv, ClassBranch* class_equiv);
    bool resolveDisjunctionTree(IndividualBranch* indiv, ClassExpression* ano_elem);

    void addInferredInheritance(IndividualBranch* indiv,
                                AnonymousClassBranch* anonymous_branch,
                                AnonymousClassTree* anonymous_tree,
                                const std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used);  

    std::string computeDebugUpdate(IndividualBranch* indiv, AnonymousClassTree* anonymous_tree)
    {
      std::string res;
      res = "indiv " + indiv->value() + ": " + indiv->updatesToString();
      res += "with " + anonymous_tree->id + ": " + anonymous_tree->involvesToString();
      if(indiv->flags_.find("equiv") != indiv->flags_.end())
        res += "|equiv: 1";
      else
        res += "|equiv: 0";
      return res;
    }

    template<typename T, typename B>
    bool resolveAllValuesFrom(const std::vector<PairElement<T*, B*>>& relations, T* target_property,
                              std::vector<std::pair<std::string, size_t>>& indexes,
                              ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
    {
      for(size_t i = 0; i < relations.size(); i++)
      {
        if(existInInheritance(relations[i].first, target_property, used))
        {
          if(resolveTree(relations[i].second, expression->sub_elements_.front(), used))
            indexes.emplace_back(relations[i].first->value() + "|" + relations[i].second->value(), i);
          else
            return false;
        }
      }
      return true;
    }

    template<typename T, typename B>
    std::pair<std::string, int> resolveSomeValuesFrom(const std::vector<PairElement<T*, B*>>& relations, T* target_property,
                                                      ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
    {
      for(size_t i = 0; i < relations.size(); i++)
        if(existInInheritance(relations[i].first, target_property, used))
          if(resolveTree(relations[i].second, expression->sub_elements_.front(), used))
            return std::make_pair(relations[i].first->value() + "|" + relations[i].second->value(), i);
      
      return std::make_pair("", -1);
    }

    template<typename T, typename B>
    std::pair<std::string, int> resolveHasValue(const std::vector<PairElement<T*, B*>>& relations, T* target_property,
                                                ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
    {
      for(size_t i = 0; i < relations.size(); i++)
        if(existInInheritance(relations[i].first, target_property, used))
          if(checkValue(relations[i].second, expression->sub_elements_.front(), used))
            return std::make_pair(relations[i].first->value() + "|" + relations[i].second->value(), i);
      
      return std::make_pair("", -1);
    }

    template<typename T, typename B>
    bool resolveMaxCardinality(const std::vector<PairElement<T*, B*>>& relations, T* target_property,
                               std::vector<std::pair<std::string, size_t>>& indexes, size_t& nb_relation,
                               ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
    {
      for(size_t i = 0; i < relations.size(); i++)
      {
        if(existInInheritance(relations[i].first, target_property, used))
        {
          if(expression->sub_elements_.empty() ||                                       // unqualified cardinality
             resolveTree(relations[i].second, expression->sub_elements_.front(), used)) // qualified cardinality
          {
            nb_relation++;
            if(nb_relation > expression->cardinality_value_)
              return false;
            else
              indexes.emplace_back(relations[i].first->value() + "|" + relations[i].second->value(), i);
          }
        }
      }
      return true;
    }

    template<typename T, typename B>
    bool resolveMinCardinality(const std::vector<PairElement<T*, B*>>& relations, T* target_property,
                               std::vector<std::pair<std::string, size_t>>& indexes, size_t& nb_relation,
                               ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
    {
      for(size_t i = 0; i < relations.size(); i++)
      {
        if(existInInheritance(relations[i].first, target_property, used))
        {
          if(expression->sub_elements_.empty() ||                                       // unqualified cardinality
             resolveTree(relations[i].second, expression->sub_elements_.front(), used)) // qualified cardinality
          {
            nb_relation++;
            indexes.emplace_back(relations[i].first->value() + "|" + relations[i].second->value(), i);
            if(nb_relation >= expression->cardinality_value_)
              return true;             
          }
        }
      }
      return false;
    }

    template<typename T>
    bool existInInheritance(T* branch, ValuedNode* selector, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
    {
      if(branch == selector)
        return true;
      else
      {
        for(size_t i = 0; i < branch->mothers_.size(); i++)
        {
          if(existInInheritance(branch->mothers_[i].elem, selector, used))
          {
            used.emplace_back(branch->value() + "|isA|" + selector->value(), branch->mothers_.has_induced_inheritance_relations[i]);
            return true;
          }
        }
      }
      return false;
    }

  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERANO_H
