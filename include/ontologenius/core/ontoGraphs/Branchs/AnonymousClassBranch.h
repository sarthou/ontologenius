#ifndef ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H
#define ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

  enum CardType_e
  {
    cardinality_none,
    cardinality_some,
    cardinality_only,
    cardinality_min,
    cardinality_max,
    cardinality_exactly,
    cardinality_value,
    cardinality_error
  };

  enum LogicalNodeType_e
  {
    logical_and,
    logical_or,
    logical_not,
    logical_none
  };

  struct CardinalityElement_t
  {
    CardType_e card_type_ = cardinality_none;
    size_t card_number_ = 0;
    LiteralNode* card_range_ = nullptr;
  };

  class AnonymousClassElement
  {
  public:
    AnonymousClassElement() : logical_type_(logical_none), oneof(false), is_complex(false),
                              class_involved_(nullptr), object_property_involved_(nullptr),
                              data_property_involved_(nullptr), individual_involved_(nullptr) {}

    LogicalNodeType_e logical_type_;
    bool oneof; // true = OneOf element
    bool is_complex;

    // pointers to the concepts used in the equivalence relation
    ClassBranch* class_involved_;
    ObjectPropertyBranch* object_property_involved_;
    DataPropertyBranch* data_property_involved_;
    IndividualBranch* individual_involved_;

    CardinalityElement_t card_;

    std::vector<AnonymousClassElement*> sub_elements_;
    std::string ano_name;
  };

  class AnonymousClassBranch : public ValuedNode
  {
  public:
    explicit AnonymousClassBranch(const std::string& value) : ValuedNode(value), class_equiv_(nullptr), depth_(0) {}

    ClassBranch* class_equiv_;
    std::vector<AnonymousClassElement*> ano_elems_;
    size_t depth_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H