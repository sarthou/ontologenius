#ifndef ONTOLOGENIUS_RELATIONSWITHINDUCTIONS_H
#define ONTOLOGENIUS_RELATIONSWITHINDUCTIONS_H

#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"

namespace ontologenius {

  class IndividualBranch;
  class ObjectPropertyBranch;
  class DataPropertyBranch;
  class ClassBranch;
  class LiteralNode;

  using ObjectRelationTriplet_t = Triplet_t<IndividualBranch, ObjectPropertyBranch, IndividualBranch>;
  using ObjectRelationTriplets = Triplets<IndividualBranch, ObjectPropertyBranch, IndividualBranch>;

  using DataRelationTriplet_t = Triplet_t<IndividualBranch, DataPropertyBranch, LiteralNode>;
  using DataRelationTriplets = Triplets<IndividualBranch, DataPropertyBranch, LiteralNode>;

  using InheritedRelationTriplet_t = Triplet_t<IndividualBranch, void, ClassBranch>;
  using InheritedRelationTriplets = Triplets<IndividualBranch, void, ClassBranch>;

  template<typename T>
  class RelationsWithInductions
  {
  public:
    std::vector<T> relations;
    std::vector<ObjectRelationTriplets*> has_induced_object_relations;
    std::vector<DataRelationTriplets*> has_induced_data_relations;
    std::vector<InheritedRelationTriplets*> has_induced_inheritance_relations;

    RelationsWithInductions() = default;
    RelationsWithInductions(const RelationsWithInductions& other) = delete;
    ~RelationsWithInductions() { clear(); }

    size_t size() const { return relations.size(); }
    bool empty() const { return relations.empty(); }
    T& operator[](size_t index) { return relations[index]; }
    const T& at(size_t index) const { return relations.at(index); }

    void resetUpdated() { updated_ = false; }
    bool isUpdated() { return updated_; }

    size_t pushBack(T& relation)
    {
      relations.emplace_back(relation);
      has_induced_object_relations.emplace_back(new ObjectRelationTriplets);
      has_induced_data_relations.emplace_back(new DataRelationTriplets);
      has_induced_inheritance_relations.emplace_back(new InheritedRelationTriplets);
      updated_ = true;
      return relations.size() - 1;
    }

    template<class... Args>
    T& emplaceBack(Args&&... args)
    {
      has_induced_object_relations.emplace_back(new ObjectRelationTriplets);
      has_induced_data_relations.emplace_back(new DataRelationTriplets);
      has_induced_inheritance_relations.emplace_back(new InheritedRelationTriplets);
      updated_ = true;
      return relations.emplace_back(std::forward<Args>(args)...);
    }

    void erase(size_t index)
    {
      relations.erase(relations.begin() + index);
      delete has_induced_object_relations.at(index);
      has_induced_object_relations.erase(has_induced_object_relations.begin() + (int)index);
      delete has_induced_data_relations.at(index);
      has_induced_data_relations.erase(has_induced_data_relations.begin() + (int)index);
      delete has_induced_inheritance_relations.at(index);
      has_induced_inheritance_relations.erase(has_induced_inheritance_relations.begin() + (int)index);
    }

    void clear()
    {
      for(auto* induced_object : has_induced_object_relations)
        delete induced_object;
      for(auto* induced_data : has_induced_data_relations)
        delete induced_data;
      for(auto* induced_inherit : has_induced_inheritance_relations)
        delete induced_inherit;

      relations.clear();
      has_induced_object_relations.clear();
      has_induced_data_relations.clear();
      has_induced_inheritance_relations.clear();
    }

    typename std::vector<T>::iterator find(T other)
    {
      return std::find(relations.begin(), relations.end(), other);
    }

    typename std::vector<T>::iterator begin() { return relations.begin(); }
    typename std::vector<T>::iterator end() { return relations.end(); }
    typename std::vector<T>::const_iterator cbegin() const { return relations.cbegin(); }
    typename std::vector<T>::const_iterator cend() const { return relations.cend(); }
    typename std::vector<T>::reverse_iterator rbegin() { return relations.rbegin(); }
    typename std::vector<T>::reverse_iterator rend() { return relations.rend(); }

    T& back() { return relations.back(); }
    T& front() { return relations.front(); }

  private:
    bool updated_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_RELATIONSWITHINDUCTIONS_H