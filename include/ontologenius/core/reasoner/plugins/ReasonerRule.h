#ifndef ONTOLOGENIUS_REASONERRULE_H
#define ONTOLOGENIUS_REASONERRULE_H

#include <tuple>
#include <unordered_map>

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  struct RuleUsedTriplet_t
  {
    RuleUsedTriplet_t(InheritedRelationTriplets* inheritance, ObjectRelationTriplets* object, DataRelationTriplets* data)
    {
      used_triplet_ = std::make_tuple(inheritance, object, data);
    }
    std::tuple<InheritedRelationTriplets*, ObjectRelationTriplets*, DataRelationTriplets*> used_triplet_;

    template<typename T>
    T getUsedTriplet(const int index)
    {
      switch(index)
      {
      case 0:
        return std::get<InheritedRelationTriplets>(used_triplet_); // return std::get<0>(used_triplet_);
        break;
      case 1:
        return std::get<ObjectRelationTriplets>(used_triplet_); // return std::get<1>(used_triplet_);
        break;
      case 2:
        return std::get<DataRelationTriplets>(used_triplet_); // return std::get<2>(used_triplet_);
        break;
      default:
        break;
      }
    }

    InheritedRelationTriplets* getInheritance()
    {
      return std::get<0>(used_triplet_);
    }

    ObjectRelationTriplets* getObject()
    {
      return std::get<1>(used_triplet_);
    }

    DataRelationTriplets* getData()
    {
      return std::get<2>(used_triplet_);
    }
  };

  struct IndivResult_t
  {
    IndivResult_t() : indiv(nullptr), literal(nullptr)
    {}

    IndividualBranch* indiv;               // for individual variable
    LiteralNode* literal;                  // for datatype variable
    std::vector<std::string> explanations; // or std::string if we concatenate every explanation member
    std::vector<RuleUsedTriplet_t> used_triplets;

    bool empty() const
    {
      if((indiv == nullptr) && (literal == nullptr))
        return true;
      else
        return false;
    }
  };

  struct RuleResult_t // represent one instantiation of a rule (variables are set to the indiv) e.g: Agent(pr2), hasCapability(pr2, pr2_capa)
  {
    std::vector<index_t> assigned_result;         // e.g : <pr2, pr2_capa>, can be either indiv or literal indexes
    std::vector<std::string> explanations;        // e.g : <pr2|isA|Robot, Robot|isA|Agent, pr2|hasCapability|pr2_capa>
    std::vector<RuleUsedTriplet_t> triplets_used; // e.g : <<&pr2.is_a_[Robot], &Robot.mothers_[Agent], &pr2.object_relations_[hasCapability : pr2_capa]>

    RuleResult_t(const std::size_t& nb_var) : assigned_result(nb_var, 0)
    {}

    void insertResult(const IndivResult_t& result, const size_t& var_index)
    {
      if(result.indiv != nullptr)
        assigned_result[var_index] = result.indiv->get();
      else if(result.literal != nullptr)
        assigned_result[var_index] = result.literal->get();

      explanations.insert(explanations.end(), result.explanations.begin(), result.explanations.end());

      triplets_used.insert(triplets_used.end(), result.used_triplets.begin(), result.used_triplets.end());
    }
  };

  class ReasonerRule : public ReasonerInterface
  {
  public:
    ReasonerRule();
    ~ReasonerRule() override = default;

    void postReason() override;
    void setParameter(const std::string& name, const std::string& value) override;

    bool implementPostReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;

    bool defaultActive() override { return true; }

  private:
    bool standard_mode_;

    std::vector<RuleResult_t> resolve(RuleBranch* rule_branch, std::vector<RuleTriplet_t>& atoms, std::vector<index_t>& accu);

    void resolveAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveClassAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveObjectAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveDataAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);

    std::vector<IndivResult_t> getFromObject(RuleTriplet_t& triplet, const index_t& index_indiv_from);
    std::vector<IndivResult_t> getFromData(RuleTriplet_t& triplet, const index_t& index_indiv_from);

    std::unordered_set<IndividualBranch*> getFrom(ObjectPropertyBranch* property, const index_t& index_indiv_on);
    std::unordered_set<IndividualBranch*> getFrom(DataPropertyBranch* property, const index_t& index_literal_on);

    std::vector<IndivResult_t> getOnObject(RuleTriplet_t& triplet, const index_t& index_indiv_on);
    std::vector<IndivResult_t> getOnData(RuleTriplet_t& triplet, const index_t& index_literal_on);

    std::unordered_set<IndividualBranch*> getOn(const index_t& index_indiv_from, ObjectPropertyBranch* predicate);
    std::unordered_set<LiteralNode*> getOn(const index_t& index_indiv_from, DataPropertyBranch* predicate);

    std::vector<IndivResult_t> getType(ClassBranch* class_selector);

    IndivResult_t checkInstantiatedTriplet(IndividualBranch* indiv, ClassBranch* class_selector);
    IndivResult_t checkInstantiatedTriplet(IndividualBranch* indiv_from, ObjectPropertyBranch* property_predicate, IndividualBranch* indiv_on, bool var_from);
    IndivResult_t checkInstantiatedTriplet(IndividualBranch* indiv_from, DataPropertyBranch* property_predicate, LiteralNode* literal_on, bool var_from);

    bool checkValue(IndividualBranch* indiv_from, IndividualBranch* indiv_on, IndivResult_t& used);
    bool checkValue(LiteralNode* literal_from, LiteralNode* literal_on, IndivResult_t& used);

    template<typename T>
    bool existInInheritance(T* branch, index_t selector, IndivResult_t& used)
    {
      std::string explanation;

      if(branch->get() == selector)
        return true;
      else
      {
        for(size_t i = 0; i < branch->mothers_.size(); i++)
        {
          if(existInInheritance(branch->mothers_[i].elem, selector, used))
          {
            explanation = branch->value() + "|isA|" + branch->mothers_[i].elem->value();
            used.explanations.emplace_back(explanation);

            used.used_triplets.emplace_back(branch->mothers_.has_induced_inheritance_relations[i],
                                            branch->mothers_.has_induced_object_relations[i],
                                            branch->mothers_.has_induced_data_relations[i]);
            return true;
          }
        }
      }
      return false;
    }

    template<typename T, typename B, typename C>
    int checkInstantiatedTriplet(IndividualBranch* indiv_from, T* property_predicate, B* resource_on, const std::vector<C>& relations, IndivResult_t& used)
    {
      std::string explanation;
      int index = -1;

      for(size_t i = 0; i < relations.size(); i++)
      {
        if(existInInheritance(relations[i].first, property_predicate->get(), used))
        {
          if(checkValue(relations[i].second, resource_on, used))
          {
            explanation = indiv_from->value() + "|" + relations[i].first->value() + "|" + relations[i].second->value();
            used.explanations.emplace_back(explanation);

            return int(i);
          }
        }
      }
      return index;
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERRULE_H