#ifndef ONTOLOGENIUS_DIFFERENCEFINDER_H
#define ONTOLOGENIUS_DIFFERENCEFINDER_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Ontology.h"

namespace ontologenius {

  class Comparator
  {
  public:
    Comparator() : concept_("") {}
    std::string concept_;
    std::vector<std::string> object_properties_name_;
    std::vector<std::string> object_properties_on_;

    std::vector<std::string> data_properties_name_;
    std::vector<std::string> data_properties_data_;

    std::vector<std::string> mothers_;
  };

  class DifferenceFinder
  {
  public:
    DifferenceFinder() = default;

    std::vector<std::string> getDiff(Ontology* onto1, Ontology* onto2, const std::string& concept);

  private:
    Comparator toComparator(IndividualBranch* indiv);
    Comparator toComparator(ClassBranch* class_);

    std::vector<std::string> compare(Comparator& comp1, Comparator& comp2);
    void compareObjects(Comparator& comp1, Comparator& comp2, std::vector<std::string>& res);
    void compareDatas(Comparator& comp1, Comparator& comp2, std::vector<std::string>& res);
    void compareMothers(Comparator& comp1, Comparator& comp2, std::vector<std::string>& res);

    std::vector<std::string> toValued(const std::vector<ObjectPropertyBranch*>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it->value(); });
      return res;
    }

    std::vector<std::string> toValued(const std::vector<DataPropertyBranch*>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it->value(); });
      return res;
    }

    std::vector<std::string> toValued(const std::vector<IndividualBranch*>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it->value(); });
      return res;
    }

    std::vector<std::string> toValued(const std::vector<ClassBranch*>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it->value(); });
      return res;
    }

    template<typename T>
    std::vector<std::string> toValued(const std::vector<SingleElement<T>>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it.elem->value(); });
      return res;
    }

    template<typename T, typename U>
    std::vector<std::string> toValuedFirst(const std::vector<PairElement<T, U>>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it.first->value(); });
      return res;
    }

    template<typename T, typename U>
    std::vector<std::string> toValuedSecond(const std::vector<PairElement<T, U>>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it.second->value(); });
      return res;
    }

    template<typename T>
    std::vector<std::string> toValued(const RelationsWithInductions<SingleElement<T>>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it.elem->value(); });
      return res;
    }

    template<typename T, typename U>
    std::vector<std::string> toValuedFirst(const RelationsWithInductions<PairElement<T, U>>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it.first->value(); });
      return res;
    }

    template<typename T, typename U>
    std::vector<std::string> toValuedSecond(const RelationsWithInductions<PairElement<T, U>>& vect)
    {
      std::vector<std::string> res;
      std::transform(vect.cbegin(), vect.cend(), std::back_inserter(res), [](auto it) { return it.second->value(); });
      return res;
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_DIFFERENCEFINDER_H
