#ifndef ONTOLOGENIUS_REASONERRULE_H
#define ONTOLOGENIUS_REASONERRULE_H

#include <tuple>
#include <unordered_map>

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  // using RuleUsedTriplet_t = Triplet_t<InheritedRelationTriplets*, ObjectRelationTriplets*, DataRelationTriplets*>;
  // using RuleUsedTriplet_t = std::tuple<InheritedRelationTriplets*, ObjectRelationTriplets*, DataRelationTriplets*>;
  //  using RuleUsedTriplets = Triplets<InheritedRelationTriplets*, ObjectRelationTriplets*, DataRelationTriplets*>;

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

  enum Evaluation_e // since we don't evaluate triplets with both subject/object
  {
    none,
    from,
    on,
    both
  };

  struct IndivResult_t // found instantiation for a variable (indiv or datatype)
  {
    IndivResult_t() : indiv(nullptr), literal(nullptr)
    {}

    IndivResult_t(IndividualBranch* indiv_res, const std::string& expl, const RuleUsedTriplet_t& triplets) : indiv(indiv_res), literal(nullptr)
    {
      explanations.push_back(expl);
      used_triplets.push_back(triplets);
    }

    IndivResult_t(LiteralNode* literal_res) : indiv(nullptr), literal(literal_res)
    {}

    // IndivResult_t(LiteralNode* lit, const std::string& expl, const RuleUsedTriplet_t& triplets)
    // {
    //   literal = lit;
    //   explanations.push_back(expl);
    //   used_triplets.push_back(triplets);
    // }

    // // to add into constructor -> explanations and used_triplets
    // IndivResult_t(IndividualBranch* indiv_branch) : indiv(indiv_branch), literal(nullptr)
    // {}

    // // to add into constructor -> explanations and used_triplets
    // IndivResult_t(LiteralNode* literal_branch) : indiv(nullptr), literal(literal_branch)
    // {}

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

  bool isSelectorDefined(const IndivResult_t& value) { return (value.indiv != nullptr) || bool(value.literal); } // if both are nullptr, then there is nothing in there (no previous instantiation)

  struct RuleResult_t // represent one instantiation of a rule (variables are set to the indiv) e.g: Agent(pr2), hasCapability(pr2, pr2_capa)
  {
    std::vector<IndividualBranch*> indivs;                     // e.g : <pr2, pr2_capa>
    std::vector<std::vector<std::string>> explanations;        // e.g : <pr2|isA|Robot, Robot|isA|Agent, pr2|hasCapability|pr2_capa>
    std::vector<std::vector<RuleUsedTriplet_t>> triplets_used; // e.g : <<&pr2.is_a_[Robot], &Robot.mothers_[Agent], &pr2.object_relations_[hasCapability : pr2_capa]>

    std::string toString() const
    {
      std::string res;
      for(const auto& ind : indivs)
        res += ind->value() + ", ";

      return res;
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

    std::vector<std::vector<IndivResult_t>> resolve(RuleBranch* rule_branch, std::vector<RuleTriplet_t>& atoms, std::vector<IndivResult_t>& accu);
    std::vector<RuleResult_t> transformResults(std::vector<std::vector<IndivResult_t>>& results);

    void resolveAtom(RuleBranch* rule_branch, RuleTriplet_t triplet, std::vector<IndivResult_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveClassAtom(RuleBranch* rule_branch, RuleTriplet_t triplet, std::vector<IndivResult_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveObjectAtom(RuleBranch* rule_branch, RuleTriplet_t triplet, std::vector<IndivResult_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveDataAtom(RuleBranch* rule_branch, RuleTriplet_t triplet, std::vector<IndivResult_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);

    std::vector<IndivResult_t> getFromObject(RuleTriplet_t& triplet, IndivResult_t& indiv_from);
    std::vector<IndivResult_t> getFromData(RuleTriplet_t& triplet, IndivResult_t& indiv_from);

    std::unordered_set<IndividualBranch*> getFrom(ObjectPropertyBranch* property, IndividualBranch* individual_on);
    std::unordered_set<IndividualBranch*> getFrom(DataPropertyBranch* property, LiteralNode* literal_on);

    std::vector<IndivResult_t> getOnObject(RuleTriplet_t& triplet, IndivResult_t& indiv_on);
    std::vector<IndivResult_t> getOnData(RuleTriplet_t& triplet, IndivResult_t& literal_on);

    std::unordered_set<IndividualBranch*> getOn(IndividualBranch* indiv_from, ObjectPropertyBranch* predicate);
    std::unordered_set<LiteralNode*> getOn(IndividualBranch* indiv_from, DataPropertyBranch* predicate);

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
            explanation = branch->value() + "|isA|" + branch->mothers_[i].elem->value() + ";";
            used.explanations.emplace_back(explanation);

            RuleUsedTriplet_t used_mother(branch->mothers_.has_induced_inheritance_relations[i],
                                          branch->mothers_.has_induced_object_relations[i],
                                          branch->mothers_.has_induced_data_relations[i]);

            used.used_triplets.emplace_back(used_mother);
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
            explanation = indiv_from->value() + "|" + relations[i].first->value() + "|" + relations[i].second->value() + ";";
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