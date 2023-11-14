#ifndef ONTOLOGENIUS_ANOCLASSBRANCH_H
#define ONTOLOGENIUS_ANOCLASSBRANCH_H

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
  LogicalNodeType_e logical_type_;
  bool andor; // true = and / false = or
  bool negation; // true = not / false = nothing
  bool oneof; // true = OneOf element
  bool is_complex;

  // pointers to the concepts used in the equivalence relation
  ClassBranch_t* class_involved_;
  ObjectPropertyBranch_t* object_property_involved_;
  DataPropertyBranch_t* data_property_involved_;
  IndividualBranch_t* individual_involved_;

  CardinalityElement_t card_;

  //AnonymousClassBranch_t* anonymous_class_; // pointer to the anonymous class (branch)
  std::vector<AnonymousClassElement_t*> sub_elements_; // vector of sub elements, if size == 0 => this element is an expression (leaf)

  AnonymousClassElement_t() : logical_type_(logical_none), andor(false), negation(false), oneof(false), nb_sub(0), class_involved_(nullptr), object_property_involved_(nullptr), 
                              data_property_involved_(nullptr), individual_involved_(nullptr) {}
 
};

class AnonymousClassBranch_t : public ValuedNode
{
public:
  AnonymousClassBranch_t(const std::string& value) : ValuedNode(value), class_equiv_(nullptr), ano_elem_(nullptr) {}  
  // pointer to the class which is equivalent to this AnonymousClass
  ClassBranch_t* class_equiv_;

  AnonymousClassElement_t* ano_elem_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOCLASSBRANCH_H