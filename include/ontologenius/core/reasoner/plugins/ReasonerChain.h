#ifndef ONTOLOGENIUS_REASONERCHAIN_H
#define ONTOLOGENIUS_REASONERCHAIN_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

typedef std::vector<std::pair<std::string, ObjectRelationTriplets*>> UsedVector;

class ReasonerChain : public ReasonerInterface
{
public:
  ReasonerChain() {}
  ~ReasonerChain() {}

  virtual void postReason() override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultActive() override {return true;}
private:
  void getUpPtrChain(ObjectPropertyBranch_t* branch, std::unordered_set<ObjectPropertyBranch_t*>& res);

  std::vector<std::pair<IndividualBranch_t*, UsedVector>> resolveChain(IndividualBranch_t* indiv, const std::vector<ObjectPropertyBranch_t*>& chain, size_t chain_index = 0);
  void resolveChain(IndividualBranch_t* indiv, int same_index, const std::vector<ObjectPropertyBranch_t*>& chain, size_t chain_index, std::vector<std::pair<IndividualBranch_t*, UsedVector>>& res);

  bool relationExists(IndividualBranch_t* indiv_on, ObjectPropertyBranch_t* chain_prop, IndividualBranch_t* chain_indiv);

  template <typename T>
  bool existInInheritance(T* branch, index_t selector, UsedVector& used)
  {
    if(branch->get() == selector)
      return true;
    else
    {
      for(size_t i = 0;  i < branch->mothers_.size() ; i++)
      {
        if(existInInheritance(branch->mothers_[i].elem, selector, used))
        {
          std::string explanation = branch->value() + "|isA|" +  branch->mothers_[i].elem->value();
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
