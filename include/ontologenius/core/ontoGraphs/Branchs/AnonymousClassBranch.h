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
  none_,
  some_,
  only_,
  min_,
  max_,
  exactly_,
  value_,
  error_
};

struct CardinalityElement_t
{
  CardType_t card_type_ = none_;
  int card_number_ = 0;
  LiteralNode* card_range_ = nullptr;
};

class AnonymousClassElement_t
{
public:
  bool andor; // true = and / false = or
  bool negation; // true = not / false = nothing
  bool oneof; // true = OneOf element
  int nb_sub; // number of sub elements

  // pointers to the concepts used in the equivalence relation
  ClassBranch_t* class_involved_;
  ObjectPropertyBranch_t* object_property_involved_;
  DataPropertyBranch_t* data_property_involved_;
  IndividualBranch_t* individual_involved_;

  CardinalityElement_t card_;

  //AnonymousClassBranch_t* anonymous_class_; // pointer to the anonymous class (branch)
  std::vector<AnonymousClassElement_t*> sub_elements_; // vector of sub elements, if size == 0 => this element is an expression (leaf)

  AnonymousClassElement_t() : andor(false), negation(false), oneof(false), nb_sub(0), class_involved_(nullptr), object_property_involved_(nullptr), 
                              data_property_involved_(nullptr), individual_involved_(nullptr) {};
 
};

class AnonymousClassBranch_t : public ValuedNode
{
public:
  // pointer to the class which is equivalent to this AnonymousClass
  ClassBranch_t* class_equiv_;
  // pointers to the concepts used in the equivalence relation
  ClassBranch_t* class_involved_;
  ObjectPropertyBranch_t* object_property_involved_;
  DataPropertyBranch_t* data_property_involved_;
  IndividualBranch_t* individual_involved_;

  CardType_t card_type_;
  int card_number_;
  LiteralNode* card_range_;

  AnonymousClassElement_t* ano_elem_;

  AnonymousClassBranch_t(const std::string& value = "") : ValuedNode(value), class_equiv_(nullptr), class_involved_(nullptr),
                                                          object_property_involved_(nullptr), data_property_involved_(nullptr), 
                                                          individual_involved_(nullptr)  {};
 
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOCLASSBRANCH_H