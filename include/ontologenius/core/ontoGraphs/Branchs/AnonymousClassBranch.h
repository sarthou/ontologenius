#ifndef ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H
#define ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"

namespace ontologenius {

enum CardType_t 
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
  CardType_t card_type_ = cardinality_none;
  size_t card_number_ = 0;
  LiteralNode* card_range_ = nullptr;
};

class AnonymousClassElement_t
{
public:
  AnonymousClassElement_t() : logical_type_(logical_none), oneof(false), is_complex(false), 
                              class_involved_(nullptr), object_property_involved_(nullptr), 
                              data_property_involved_(nullptr), individual_involved_(nullptr) {}
                              
  LogicalNodeType_e logical_type_;
  bool oneof; // true = OneOf element
  bool is_complex;

  // pointers to the concepts used in the equivalence relation
  ClassBranch_t* class_involved_;
  ObjectPropertyBranch_t* object_property_involved_;
  DataPropertyBranch_t* data_property_involved_;
  IndividualBranch_t* individual_involved_;

  CardinalityElement_t card_;

  std::vector<AnonymousClassElement_t*> sub_elements_;
  std::string ano_name;
};

class AnonymousClassBranches_t : public ValuedNode
{
public:
  explicit AnonymousClassBranches_t(const std::string& value) : ValuedNode(value), class_equiv_(nullptr), depth_(0) {}  

  ClassBranch_t* class_equiv_;
  std::vector<AnonymousClassElement_t*> ano_elems_;
  size_t depth_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H