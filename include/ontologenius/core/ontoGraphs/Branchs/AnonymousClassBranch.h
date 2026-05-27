#ifndef ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H
#define ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

  enum RestrictionConstraintType_e
  {
    restriction_all_values_from,
    restriction_some_values_from,
    restriction_has_value,
    restriction_max_cardinality,
    restriction_min_cardinality,
    restriction_cardinality,
    restriction_unknown
  };

  enum ClassExpressionType_e
  {
    class_expression_identifier,      // type 1
    class_expression_one_of,          // type 2
    class_expression_restriction,     // type 3
    class_expression_intersection_of, // type 4
    class_expression_union_of,        // type 5
    class_expression_complement_of,   // type 6
    class_expression_unknown
  };

  class AnonymousClassTree;

  class ClassExpression
  {
  public:
    ClassExpression() : type_(class_expression_unknown),
                        class_involved_(nullptr), object_property_involved_(nullptr),
                        data_property_involved_(nullptr), individual_involved_(nullptr),
                        literal_involved_(nullptr), datatype_involved_(nullptr),
                        restriction_type_(restriction_unknown), cardinality_value_(0)
    {}

    ClassExpressionType_e type_;

    // pointers to the concepts used in the equivalence relation
    ClassBranch* class_involved_;
    ObjectPropertyBranch* object_property_involved_;
    DataPropertyBranch* data_property_involved_;
    IndividualBranch* individual_involved_;
    LiteralNode* literal_involved_;
    LiteralType* datatype_involved_;

    RestrictionConstraintType_e restriction_type_;
    size_t cardinality_value_;

    std::vector<ClassExpression*> sub_elements_;
  };

  class AnonymousClassTree : public InferenceRuleNode
  {
  public:
    AnonymousClassTree(const std::string& rule) : InferenceRuleNode(rule),
                                                  involves_class(false), involves_object_property(false), involves_data_property(false), involves_individual(false),
                                                  involves_close_world_assumption(false), root_node_(nullptr),
                                                  depth_(0)
    {}

    bool involves_class;
    bool involves_object_property;
    bool involves_data_property;
    bool involves_individual;
    bool involves_close_world_assumption;

    ClassExpression* root_node_;
    std::unordered_set<ClassBranch*> proved_classes_;                                          // Class assertions immediately provable on the initial individual
    std::vector<std::pair<ObjectPropertyBranch*, IndividualBranch*>> proved_object_relations_; // Object property assertions with a fully determined target (from hasValue)
    std::vector<std::pair<DataPropertyBranch*, LiteralNode*>> proved_data_relations_;          // Data property assertions with a fully determined literal target (from hasValue)
    std::unordered_set<ObjectPropertyBranch*> initial_required_object_properties_;             // Properties the initial individual must hold (value undetermined)
    std::unordered_set<DataPropertyBranch*> initial_required_data_properties_;                 // Properties the initial individual must hold (value undetermined)
    size_t depth_;

    std::string id;

    std::string involvesToString() const
    {
      std::string involves_res;
      involves_res = " c : " + std::to_string(static_cast<int>(involves_class)) + " o : " + std::to_string(static_cast<int>(involves_object_property)) +
                     " d : " + std::to_string(static_cast<int>(involves_data_property)) + " i : " + std::to_string(static_cast<int>(involves_individual)) +
                     " cwa : " + std::to_string(static_cast<int>(involves_close_world_assumption));
      return involves_res;
    }
  };

  class AnonymousClassBranch : public ValuedNode
  {
  public:
    explicit AnonymousClassBranch(const std::string& value, bool hidden = false) : ValuedNode(value, hidden), class_equiv_(nullptr) {}
    AnonymousClassBranch(const AnonymousClassBranch& other) = delete;
    ~AnonymousClassBranch()
    {
      for(auto* tree : ano_trees_)
        delete tree;
    }

    ClassBranch* class_equiv_;
    std::vector<AnonymousClassTree*> ano_trees_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANONYMOUSCLASSBRANCH_H