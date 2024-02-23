#ifndef ONTOLOGENIUS_REASONERTRANSITIVITY_H
#define ONTOLOGENIUS_REASONERTRANSITIVITY_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

typedef std::vector<std::pair<std::string, ObjectRelationTriplets*>> UsedVector;

class ChainTree;
class ReasonerTransitivity : public ReasonerInterface
{
public:
  ReasonerTransitivity() {}
  ~ReasonerTransitivity() {}

  virtual void postReason() override;

  virtual bool implementPostReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultActive() override {return true;}
private:
  void getUpPtrTransitive(ObjectPropertyBranch_t* branch, std::unordered_set<ObjectPropertyBranch_t*>& res);
  std::vector<std::pair<IndividualBranch_t*, UsedVector>> resolveChain(IndividualBranch_t* indiv, ObjectPropertyBranch_t* property, size_t current_length);
  void resolveChain(IndividualBranch_t* indiv, int same_index, ObjectPropertyBranch_t* property, size_t current_length, std::vector<std::pair<IndividualBranch_t*, UsedVector>>& res);

  bool relationExists(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on);

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

#endif // ONTOLOGENIUS_REASONERTRANSITIVITY_H
