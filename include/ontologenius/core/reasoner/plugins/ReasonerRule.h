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

    std::unordered_map<ClassBranch*, std::unordered_set<ClassBranch*>> disjoints_cache_; // copy from ReasonerAnonymous

    bool checkClassesDisjointess(IndividualBranch* indiv, ClassBranch* class_equiv);

    void resolveHead(const std::vector<RuleTriplet_t>& atoms, RuleResult_t& solution, RuleBranch* rule);
    void addInferredClassAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule);
    void addInferredObjectAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule);
    void addInferredDataAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule);

    std::vector<RuleResult_t> resolveBody(RuleBranch* rule_branch, std::vector<RuleTriplet_t>& atoms, std::vector<index_t>& accu);
    void resolveAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveClassAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveObjectAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);
    void resolveDataAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values);

    void getType(ClassBranch* class_selector, std::vector<IndivResult_t>& res, IndivResult_t prev = IndivResult_t());
    IndivResult_t isA(IndividualBranch* indiv, ClassBranch* class_selector);

    std::vector<IndivResult_t> getFromObject(RuleTriplet_t& triplet);
    std::vector<IndivResult_t> getOnObject(RuleTriplet_t& triplet, index_t selector);

    std::vector<IndivResult_t> getFromData(RuleTriplet_t& triplet);
    std::vector<IndivResult_t> getOnData(RuleTriplet_t& triplet, index_t selector);

    void constructResult(const std::string& concept,
                         const RelationsWithInductions<IndividualElement>& relation,
                         size_t index, IndivResult_t& res);

    template<typename T>
    void constructResult(const std::string& concept,
                         const RelationsWithInductions<SingleElement<T*>>& relation,
                         size_t index, IndivResult_t& res)
    {
      if(relation.at(index).elem->isHidden() == false)
      {
        std::string explanation = concept + "|isA|" + relation.at(index).elem->value();
        res.explanations.emplace_back(explanation);
      }
      else
      {
        const auto& hidden_explanation = relation.at(index).explanation;
        res.explanations.insert(res.explanations.end(),
                                hidden_explanation.cbegin(),
                                hidden_explanation.cend());
      }

      res.used_triplets.emplace_back(relation.has_induced_inheritance_relations[index],
                                     relation.has_induced_object_relations[index],
                                     relation.has_induced_data_relations[index]);
    }

    template<typename T, typename D>
    void constructResult(const std::string& concept,
                         const RelationsWithInductions<PairElement<T*, D*>>& relation,
                         size_t index, IndivResult_t& res)
    {
      std::string explanation = concept + "|" + relation.at(index).first->value() + "|" + relation.at(index).second->value();
      res.explanations.emplace_back(explanation);

      res.used_triplets.emplace_back(relation.has_induced_inheritance_relations[index],
                                     relation.has_induced_object_relations[index],
                                     relation.has_induced_data_relations[index]);
    }

    template<typename T>
    bool isA(const std::string& concept,
             T* selector,
             const RelationsWithInductions<SingleElement<T*>>& relations,
             IndivResult_t& res)
    {
      for(size_t i = 0; i < relations.size(); i++)
      {
        if(relations.at(i).elem == selector)
        {
          constructResult(concept, relations, i, res);
          return true;
        }
        else if(isA(relations.at(i).elem->value(), selector, relations.at(i).elem->mothers_, res))
        {
          constructResult(concept, relations, i, res);
          return true;
        }
      }

      return false;
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERRULE_H