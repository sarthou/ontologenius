#ifndef ONTOLOGENIUS_REASONERCHAIN_H
#define ONTOLOGENIUS_REASONERCHAIN_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  using UsedVector = std::vector<std::pair<std::string, ObjectRelationTriplets*>>;

  class ReasonerChain : public ReasonerInterface
  {
  public:
    ReasonerChain() = default;
    ~ReasonerChain() override = default;

    void postReason() override;

    bool implementPostReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;

    bool defaultActive() override { return true; }

  private:
    void getUpPtrChain(ObjectPropertyBranch* branch, std::unordered_set<ObjectPropertyBranch*>& res);

    std::vector<std::pair<IndividualBranch*, UsedVector>> resolveChain(IndividualBranch* indiv, const std::vector<ObjectPropertyBranch*>& chain, size_t chain_index = 0);
    void resolveChain(IndividualBranch* indiv, int same_index, const std::vector<ObjectPropertyBranch*>& chain, size_t chain_index, std::vector<std::pair<IndividualBranch*, UsedVector>>& res);

    template<typename T>
    bool existInInheritance(T* branch, index_t selector, UsedVector& used)
    {
      if(branch->get() == selector)
        return true;
      else
      {
        for(size_t i = 0; i < branch->mothers_.size(); i++)
        {
          if(existInInheritance(branch->mothers_[i].elem, selector, used))
          {
            std::string explanation = branch->value() + "|isA|" + branch->mothers_[i].elem->value();
            used.emplace_back(explanation, branch->mothers_.has_induced_object_relations[i]);
            return true;
          }
        }
      }
      return false;
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERCHAIN_H
