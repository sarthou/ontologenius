#ifndef ONTOLOGENIUS_PROPERTYBRANCH_H
#define ONTOLOGENIUS_PROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"

namespace ontologenius {

struct Properties_t
{
  bool functional_property_;
  bool inverse_functional_property_;
  bool transitive_property_;
  bool symetric_property_;
  bool antisymetric_property_;
  bool reflexive_property_;
  bool irreflexive_property_;

  Properties_t() : functional_property_(false),
                    inverse_functional_property_(false),
                    transitive_property_(false),
                    symetric_property_(false),
                    antisymetric_property_(false),
                    reflexive_property_(false),
                    irreflexive_property_(false) {};

    void apply(const Properties_t& other)
    {
      functional_property_ = functional_property_ || other.functional_property_;
      inverse_functional_property_ = inverse_functional_property_ || other.inverse_functional_property_;
      transitive_property_ = transitive_property_ || other.transitive_property_;
      symetric_property_ = symetric_property_ || other.symetric_property_;
      antisymetric_property_ = antisymetric_property_ || other.antisymetric_property_;
      reflexive_property_ = reflexive_property_ || other.reflexive_property_;
      irreflexive_property_ = irreflexive_property_ || other.irreflexive_property_;
    }
};

template <typename B>
class PropertyBranch_t
{
public:
  Properties_t properties_;
  bool annotation_usage_;

  PropertyBranch_t() : annotation_usage_(false) {}
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_PROPERTYBRANCH_H
