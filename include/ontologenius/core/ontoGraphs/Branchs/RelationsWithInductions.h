#ifndef ONTOLOGENIUS_RELATIONSWITHINDUCTIONS_H
#define ONTOLOGENIUS_RELATIONSWITHINDUCTIONS_H

#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"

namespace ontologenius {

  class IndividualBranch;
  class ObjectPropertyBranch;
  class ClassBranch;

  using ObjectRelationTriplet_t = Triplet_t<IndividualBranch, ObjectPropertyBranch, IndividualBranch>;
  using ObjectRelationTriplets = Triplets<IndividualBranch, ObjectPropertyBranch, IndividualBranch>;
  using InheritedRelationTriplet_t = Triplet_t<IndividualBranch, void, ClassBranch>;
  using InheritedRelationTriplets = Triplets<IndividualBranch, void, ClassBranch>;

  template<typename T>
  class RelationsWithInductions
  {
  public:
    std::vector<T> relations;
    std::vector<ObjectRelationTriplets*> has_induced_object_relations;
    std::vector<InheritedRelationTriplets*> has_induced_inheritance_relations;

    RelationsWithInductions() = default;
    RelationsWithInductions(const RelationsWithInductions& other) = delete;
    ~RelationsWithInductions() { clear(); }

    inline size_t size() const { return relations.size(); }
    inline bool empty() const { return relations.empty(); }
    T& operator[](size_t index) { return relations[index]; }

    size_t push_back(T& relation)
    {
      relations.emplace_back(relation);
      has_induced_object_relations.emplace_back(new ObjectRelationTriplets);
      has_induced_inheritance_relations.emplace_back(new InheritedRelationTriplets);
      return relations.size() - 1;
    }

    template<class... Args>
    T& emplace_back(Args&&... args)
    {
      has_induced_object_relations.emplace_back(new ObjectRelationTriplets);
      has_induced_inheritance_relations.emplace_back(new InheritedRelationTriplets);
      return relations.emplace_back(std::forward<Args>(args)...);
    }

    void erase(size_t index)
    {
      relations.erase(relations.begin() + index);
      delete has_induced_object_relations.at(index);
      has_induced_object_relations.erase(has_induced_object_relations.begin() + (int)index);
      delete has_induced_inheritance_relations.at(index);
      has_induced_inheritance_relations.erase(has_induced_inheritance_relations.begin() + (int)index);
    }

    void clear()
    {
      for(auto* induced_object : has_induced_object_relations)
        delete induced_object;
      for(auto* induced_inherit : has_induced_inheritance_relations)
        delete induced_inherit;

      relations.clear();
      has_induced_object_relations.clear();
      has_induced_inheritance_relations.clear();
    }

    typename std::vector<T>::iterator find(T other)
    {
      return std::find(relations.begin(), relations.end(), other);
    }

    inline typename std::vector<T>::iterator begin() { return relations.begin(); }
    inline typename std::vector<T>::iterator end() { return relations.end(); }
    inline typename std::vector<T>::const_iterator cbegin() const { return relations.cbegin(); }
    inline typename std::vector<T>::const_iterator cend() const { return relations.cend(); }
    inline typename std::vector<T>::reverse_iterator rbegin() { return relations.rbegin(); }
    inline typename std::vector<T>::reverse_iterator rend() { return relations.rend(); }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_RELATIONSWITHINDUCTIONS_H