#include "ontologenius/core/reasoner/plugins/ReasonerAnonymous.h"

#include <algorithm>
#include <cstddef>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"
#include "ontologenius/graphical/Display.h"

// #define DEBUG

namespace ontologenius {

  ReasonerAnonymous::ReasonerAnonymous() : standard_mode_(false)
  {}

  void ReasonerAnonymous::setParameter(const std::string& name, const std::string& value)
  {
    if(name == "standard_mode" && value == "true")
      standard_mode_ = true;
  }

  void ReasonerAnonymous::postReason()
  {
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individuals_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_class(ontology_->classes_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_obj_prop(ontology_->object_properties_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_data_prop(ontology_->data_properties_.mutex_);

    std::vector<std::pair<std::string, InheritedRelationTriplets*>> used;

    for(auto* indiv : ontology_->individuals_.get())
    {
      current_individual_ = indiv;

      if(first_run_ ||
         indiv->isUpdated() ||
         (indiv->flags_.find("equiv") != indiv->flags_.end()) ||
         indiv->hasUpdatedObjectRelation() ||    // test if one of its `object_relations` property has been updated
         indiv->hasUpdatedDataRelation() ||      // test if one of its `data_relations` property has been updated
         indiv->hasUpdatedInheritanceRelation()) // test if one of its `is_a` class has been updated
      {
        bool has_active_equiv = false;

        // Loop over every classes which includes equivalence relations
        for(auto* anonymous_branch : ontology_->anonymous_classes_.get())
        {
          if(anonymous_branch->is_equivalence_ == false)
            continue; // subClass expressions drive proved facts only, not classification
          bool trees_evaluation_result = false;
          bool has_been_evaluated = false;
          bool is_already_a = std::any_of(indiv->is_a_.cbegin(), indiv->is_a_.cend(), [anonymous_branch](const auto& is_a) { return is_a.elem == anonymous_branch->class_equiv_; }); // same as not done // need to test if same_as is_already_a

          if(is_already_a || checkClassesDisjointess(indiv, anonymous_branch->class_equiv_) == false)
          {
            for(auto* anonymous_tree : anonymous_branch->ano_trees_)
            {
#ifdef DEBUG
              computeDebugUpdate(indiv, anonymous_tree);
#endif
              std::string equiv_flag = "equiv_" + anonymous_tree->id;
              bool should_be_evaluated = true;

              if(is_already_a)
              {
                const bool inferred_by_me = std::any_of(indiv->is_a_.cbegin(), indiv->is_a_.cend(), [anonymous_branch, anonymous_tree](const auto& is_a) {
                  return ((is_a.elem == anonymous_branch->class_equiv_) && (is_a.used_rule == anonymous_tree));
                });
                if(inferred_by_me == false) // This branch has already been inferred but by another tree of the same equivalent class
                {
                  indiv->flags_[equiv_flag] = {}; // We just mark the flag to force a potential futur verification
                  should_be_evaluated = false;    // so could think it would be better to set trees_evaluation_result at true but its scope is bigger
                }
                else if(standard_mode_)
                  should_be_evaluated = false;
                else if(anonymous_tree->involves_close_world_assumption == false)
                  should_be_evaluated = false;
              }

              if(should_be_evaluated)
              {
                if((indiv->flags_.find(equiv_flag) != indiv->flags_.end()) || // has been proven to use other individuals
                   ((indiv->isUpdated() == true) &&
                    ((anonymous_tree->involves_class && indiv->is_a_.isUpdated()) ||
                     (anonymous_tree->involves_object_property && indiv->object_relations_.isUpdated()) ||
                     (anonymous_tree->involves_data_property && indiv->data_relations_.isUpdated()) ||
                     (anonymous_tree->involves_individual && (indiv->same_as_.isUpdated() || first_run_)) ||
                     (anonymous_tree->involves_close_world_assumption)))) // improve this condition to make it more specific
                {
                  has_involved_other_individual_ = false; // reset the flag before running resolveTree
                  used.clear();
                  used.reserve(anonymous_tree->depth_);
                  bool equivalence_found = resolveTree(indiv, anonymous_tree->root_node_, used);

                  trees_evaluation_result = trees_evaluation_result || equivalence_found; // mark if a least one of the tree has succeeded
                  has_been_evaluated = true;

                  if(equivalence_found)
                  {
                    has_active_equiv = true;

                    if(is_already_a == false) // the indiv is checked to still be of the same class so we can break out of the loop
                    {
                      addInferredInheritance(indiv, anonymous_branch, anonymous_tree, used);
                      is_already_a = true;
                      nb_update++;
                      if(anonymous_branch->class_equiv_->isHidden() == false)
                      {
                        explanations_.emplace_back("[ADD]" + indiv->value() + "|isA|" + anonymous_branch->class_equiv_->value(),
                                                   "[ADD] " + indiv->is_a_.back().getExplanation());
                      }
                    }
                  }

                  if(has_involved_other_individual_) // this is a pre-activation not yet complete, so we have to evaluate it next time
                  {
                    indiv->flags_[equiv_flag] = {};
                    has_active_equiv = true;
                  }
                  else
                    indiv->flags_.erase(equiv_flag);
                }
              }
            } // for all trees
          }

          // used to remove inheritance in case an individual previously inferred does not check any of the expressions after updates
          if(has_been_evaluated && is_already_a && (trees_evaluation_result == false))
          {
            indiv->nb_updates_++;
            anonymous_branch->class_equiv_->nb_updates_++;
            ontology_->individuals_.removeInheritage(indiv, anonymous_branch->class_equiv_, explanations_, true);
            // todo: no explanation provided
          }
        }

        if(has_active_equiv)
          indiv->flags_["equiv"] = {};
        else
          indiv->flags_.erase("equiv");
      }
    }

    first_run_ = false;
  }

  void ReasonerAnonymous::addInferredInheritance(IndividualBranch* indiv,
                                                 AnonymousClassBranch* anonymous_branch,
                                                 AnonymousClassTree* anonymous_tree,
                                                 const std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    const size_t new_idx = ontology_->individuals_.addClassAssertion(indiv, anonymous_branch->class_equiv_, 1.0, true, &explanations_);
    indiv->is_a_[new_idx].used_rule = anonymous_tree;

    indiv->nb_updates_++;
    anonymous_branch->class_equiv_->nb_updates_++;

    for(const auto& induced_vector : used)
    {
      indiv->is_a_[new_idx].explanation.push_back(induced_vector.first);
      // check for nullptr because OneOf returns a (string, nullptr)
      if(induced_vector.second != nullptr)
      {
        if(induced_vector.second->exist(indiv, nullptr, anonymous_branch->class_equiv_) == false)
        {
          induced_vector.second->push(indiv, nullptr, anonymous_branch->class_equiv_);
          indiv->is_a_.relations[new_idx].induced_traces.emplace_back(induced_vector.second);
        }
      }
    }
  }

  /* Full Datatype tree resolution */

  bool ReasonerAnonymous::resolveTree(LiteralNode* literal, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    switch(expression->type_)
    {
    case ClassExpressionType_e::class_expression_identifier: // Type 1
      return resolveIdentifier(literal, expression, used);

    case ClassExpressionType_e::class_expression_one_of:      // Type 2
      return resolveOneOfDatatype(literal, expression, used); // can be returned directly as `used` if not filled if false

    case ClassExpressionType_e::class_expression_intersection_of: // Type 4
      for(auto* elem : expression->sub_elements_)
      {
        if(resolveTree(literal, elem, used) == false)
        {
          used.clear();
          return false;
        }
      }
      return true;

    case ClassExpressionType_e::class_expression_union_of: // Type 5
      for(auto* elem : expression->sub_elements_)
        if(resolveTree(literal, elem, used))
          return true;
      break;

    case ClassExpressionType_e::class_expression_complement_of: // Type 6
      if(resolveTree(literal, expression->sub_elements_.front(), used) == false)
        return true;
      break;

    default: // Type 3 is not applicable to datatype
      break;
    }

    used.clear();
    return false;
  }

  // Type 1 on datatype
  bool ReasonerAnonymous::resolveIdentifier(LiteralNode* literal, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    (void)used;
    if(expression->literal_involved_ != nullptr)
      return (literal->type_ == expression->literal_involved_->type_);
    else if(expression->datatype_involved_ != nullptr)
      return (literal->type_ == expression->datatype_involved_);
    else
      return false;
  }

  // Type 2 on datatype
  bool ReasonerAnonymous::resolveOneOfDatatype(LiteralNode* literal, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::string list_string;
    std::string explanation;
    for(auto* elem : expression->sub_elements_)
    {
      if(literal == elem->literal_involved_)
        explanation = literal->value();

      if(list_string.empty() == false)
        list_string += ", ";
      list_string += elem->literal_involved_->value();
    }

    if(explanation.empty() == false)
    {
      explanation += "|isOneOf|(" + list_string + ")";
      used.emplace_back(explanation, nullptr);
      return true;
    }
    return false;
  }

  /* Full Individual tree resolution */

  bool ReasonerAnonymous::resolveTree(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    has_involved_other_individual_ = has_involved_other_individual_ || (indiv != current_individual_);

    switch(expression->type_)
    {
    case ClassExpressionType_e::class_expression_identifier: // Type 1
      return resolveIdentifier(indiv, expression, used);     // can be returned directly as `used` if not filled if false

    case ClassExpressionType_e::class_expression_one_of:      // Type 2
      return resolveOneOfIndividual(indiv, expression, used); // can be returned directly as `used` if not filled if false

    case ClassExpressionType_e::class_expression_restriction: // Type 3
      return resolveRestriction(indiv, expression, used);

    case ClassExpressionType_e::class_expression_intersection_of: // Type 4
      for(auto* elem : expression->sub_elements_)
      {
        if(resolveTree(indiv, elem, used) == false)
        {
          used.clear();
          return false;
        }
      }
      return true;

    case ClassExpressionType_e::class_expression_union_of: // Type 5
      for(auto* elem : expression->sub_elements_)
        if(resolveTree(indiv, elem, used))
          return true;
      break;

    case ClassExpressionType_e::class_expression_complement_of: // Type 6
      if(standard_mode_ == true)
        return resolveDisjunctionTree(indiv, expression->sub_elements_.front());
      else if(resolveTree(indiv, expression->sub_elements_.front(), used) == false)
        return true;
      break;

    default:
      break;
    }

    used.clear();
    return false;
  }

  // Type 1 on individuals
  bool ReasonerAnonymous::resolveIdentifier(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    if(expression->individual_involved_ != nullptr)
      return compareIndividuals(indiv, expression->individual_involved_, used);
    else if(indiv->same_as_.empty() == false)
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
      {
        if(indiv->same_as_[i].elem != indiv)
        {
          const size_t is_a_size = indiv->same_as_[i].elem->is_a_.size();
          for(size_t j = 0; j < is_a_size; j++)
          {
            if(existInInheritance(indiv->same_as_[i].elem->is_a_[j].elem, expression->class_involved_, used))
            {
              // used.emplace_back(indiv->same_as_[i].elem->value() + "|isA|" + expression->class_involved_->value(),
              //                   indiv->same_as_[i].elem->is_a_.has_induced_inheritance_relations[j]);
              used.emplace_back(indiv->same_as_[i].elem->value() + "|isA|" + indiv->same_as_[i].elem->is_a_[j].elem->value(),
                                indiv->same_as_[i].elem->is_a_.has_induced_inheritance_relations[j]);

              used.emplace_back(indiv->value() + "|sameAs|" + indiv->same_as_[i].elem->value(),
                                indiv->same_as_.has_induced_inheritance_relations[i]);
              return true;
            }
          }
        }
      }
    }
    else
    {
      for(size_t i = 0; i < indiv->is_a_.size(); i++)
        if(existInInheritance(indiv->is_a_[i].elem, expression->class_involved_, used))
        {
          // used.emplace_back(indiv->value() + "|isA|" + expression->class_involved_->value(),
          //                   indiv->is_a_.has_induced_inheritance_relations[i]);
          used.emplace_back(indiv->value() + "|isA|" + indiv->is_a_[i].elem->value(),
                            indiv->is_a_.has_induced_inheritance_relations[i]);
          return true;
        }
    }

    used.clear();
    return false;
  }

  // Type 2 on individuals
  bool ReasonerAnonymous::resolveOneOfIndividual(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    has_involved_other_individual_ = true;
    std::string list_string;
    std::string explanation;
    for(auto* elem : expression->sub_elements_)
    {
      if(compareIndividuals(indiv, elem->individual_involved_, used))
        explanation = indiv->value();

      if(list_string.empty() == false)
        list_string += ", ";
      list_string += elem->individual_involved_->value();
    }

    if(explanation.empty() == false)
    {
      explanation += "|isOneOf|(" + list_string + ")";
      used.emplace_back(explanation, nullptr);
      return true;
    }
    return false;
  }

  // Type 3 on individuals
  bool ReasonerAnonymous::resolveRestriction(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    bool solved = false;
    switch(expression->restriction_type_)
    {
    case RestrictionConstraintType_e::restriction_all_values_from:
      solved = standard_mode_ ? false : resolveAllValuesFrom(indiv, expression, used);
      break;
    case RestrictionConstraintType_e::restriction_some_values_from:
      solved = resolveSomeValuesFrom(indiv, expression, used);
      break;
    case RestrictionConstraintType_e::restriction_has_value:
      solved = resolveHasValue(indiv, expression, used);
      break;
    case RestrictionConstraintType_e::restriction_max_cardinality:
      solved = standard_mode_ ? false : resolveMaxCardinality(indiv, expression, used);
      break;
    case RestrictionConstraintType_e::restriction_min_cardinality:
      solved = standard_mode_ ? false : resolveMinCardinality(indiv, expression, used);
      break;
    case RestrictionConstraintType_e::restriction_cardinality:
      solved = standard_mode_ ? false : resolveMaxCardinality(indiv, expression, used); // Max cardinality check if it is an exact cardinality
      break;
    default:
      Display::error("Cardinality type outside of [min, max, exactly, only, value, some]");
      break;
    }

    if(solved == false)
      used.clear();
    return solved;
  }

  bool ReasonerAnonymous::compareIndividuals(IndividualBranch* indiv, IndividualBranch* other, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    if(indiv->same_as_.empty())
      return (indiv == other);
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
      {
        if((indiv->same_as_[i].elem != indiv) &&
           (indiv->same_as_[i].elem == other))
        {
          used.emplace_back(indiv->value() + "|sameAs|" + other->value(),
                            indiv->same_as_.has_induced_inheritance_relations[i]);
          return true;
        }
      }
      return false;
    }
  }

  bool ReasonerAnonymous::checkValue(IndividualBranch* indiv_from, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    has_involved_other_individual_ = has_involved_other_individual_ || (indiv_from != current_individual_); // this has to be done just because a "same_as" could be added dynamicaly
    return compareIndividuals(indiv_from, expression->individual_involved_, used);
  }

  bool ReasonerAnonymous::checkValue(LiteralNode* literal_from, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    (void)used;
    return (literal_from->get() == expression->literal_involved_->get());
  }

  bool ReasonerAnonymous::resolveAllValuesFrom(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::unordered_map<IndividualBranch*, std::vector<std::pair<std::string, size_t>>> indiv_indexes;
    bool valid = false;
    size_t nb_relations = 0;
    if(indiv->same_as_.empty())
    {
      if(expression->object_property_involved_ != nullptr)
        valid = resolveAllValuesFrom(indiv->object_relations_.relations, expression->object_property_involved_, indiv_indexes[indiv], expression, used);
      else if(expression->data_property_involved_ != nullptr)
        valid = resolveAllValuesFrom(indiv->data_relations_.relations, expression->data_property_involved_, indiv_indexes[indiv], expression, used);

      if(valid)
        nb_relations += indiv_indexes[indiv].size();
    }
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
      {
        auto* same = indiv->same_as_[i].elem;
        if(expression->object_property_involved_ != nullptr)
          valid = resolveAllValuesFrom(same->object_relations_.relations, expression->object_property_involved_, indiv_indexes[same], expression, used);
        else if(expression->data_property_involved_ != nullptr)
          valid = resolveAllValuesFrom(same->data_relations_.relations, expression->data_property_involved_, indiv_indexes[same], expression, used);

        if(valid)
        {
          nb_relations += indiv_indexes[same].size();

          if((indiv_indexes[same].empty() == false) && (same != indiv))
            used.emplace_back(indiv->value() + "|sameAs|" + same->value(), indiv->same_as_.has_induced_inheritance_relations[i]);
        }
        else
          break;
      }
    }

    if(valid && (nb_relations != 0))
    {
      if(expression->object_property_involved_ != nullptr)
      {
        for(auto& indexes : indiv_indexes)
          for(auto& expl : indexes.second)
            used.emplace_back(indexes.first->value() + "|" + expl.first, indexes.first->object_relations_.has_induced_inheritance_relations[expl.second]);
      }
      else if(expression->data_property_involved_ != nullptr)
      {
        for(auto& indexes : indiv_indexes)
          for(auto& expl : indexes.second)
            used.emplace_back(indexes.first->value() + "|" + expl.first, indexes.first->data_relations_.has_induced_inheritance_relations[expl.second]);
      }
      return true;
    }
    return false; // used will be cleared by resolveRestriction
  }

  bool ReasonerAnonymous::resolveSomeValuesFrom(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::pair<std::string, int> index{"", -1};
    IndividualBranch* valid_indiv = nullptr;

    if(expression->object_property_involved_ != nullptr)
      index = resolveSomeValuesFrom(indiv->object_relations_.relations, expression->object_property_involved_, expression, used);
    else if(expression->data_property_involved_ != nullptr)
      index = resolveSomeValuesFrom(indiv->data_relations_.relations, expression->data_property_involved_, expression, used);

    if(index.second != -1)
      valid_indiv = indiv;
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
      {
        auto* same = indiv->same_as_[i].elem;
        if(indiv != same)
        {
          if(expression->object_property_involved_ != nullptr)
            index = resolveSomeValuesFrom(same->object_relations_.relations, expression->object_property_involved_, expression, used);
          else if(expression->data_property_involved_ != nullptr)
            index = resolveSomeValuesFrom(same->data_relations_.relations, expression->data_property_involved_, expression, used);

          if(index.second != -1)
          {
            valid_indiv = same;
            used.emplace_back(indiv->value() + "|sameAs|" + same->value(), indiv->same_as_.has_induced_inheritance_relations[i]);
            break;
          }
        }
      }
    }

    if(valid_indiv != nullptr)
    {
      if(expression->object_property_involved_ != nullptr)
        used.emplace_back(valid_indiv->value() + "|" + index.first, valid_indiv->object_relations_.has_induced_inheritance_relations[index.second]);
      else if(expression->data_property_involved_ != nullptr)
        used.emplace_back(valid_indiv->value() + "|" + index.first, valid_indiv->data_relations_.has_induced_inheritance_relations[index.second]);

      return true;
    }
    return false; // used will be cleared by resolveRestriction
  }

  bool ReasonerAnonymous::resolveHasValue(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::pair<std::string, int> index{"", -1};
    IndividualBranch* valid_indiv = nullptr;

    if(expression->object_property_involved_ != nullptr)
      index = resolveHasValue(indiv->object_relations_.relations, expression->object_property_involved_, expression, used);
    else if(expression->data_property_involved_ != nullptr)
      index = resolveHasValue(indiv->data_relations_.relations, expression->data_property_involved_, expression, used);

    if(index.second != -1)
      valid_indiv = indiv;
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
      {
        auto* same = indiv->same_as_[i].elem;
        if(indiv != same)
        {
          if(expression->object_property_involved_ != nullptr)
            index = resolveHasValue(same->object_relations_.relations, expression->object_property_involved_, expression, used);
          else if(expression->data_property_involved_ != nullptr)
            index = resolveHasValue(same->data_relations_.relations, expression->data_property_involved_, expression, used);

          if(index.second != -1)
          {
            valid_indiv = same;
            used.emplace_back(indiv->value() + "|sameAs|" + same->value(), indiv->same_as_.has_induced_inheritance_relations[i]);
            break;
          }
        }
      }
    }

    if(valid_indiv != nullptr)
    {
      if(expression->object_property_involved_ != nullptr)
        used.emplace_back(valid_indiv->value() + "|" + index.first, valid_indiv->object_relations_.has_induced_inheritance_relations[index.second]);
      else if(expression->data_property_involved_ != nullptr)
        used.emplace_back(valid_indiv->value() + "|" + index.first, valid_indiv->data_relations_.has_induced_inheritance_relations[index.second]);

      return true;
    }
    return false; // used will be cleared by resolveRestriction
  }

  bool ReasonerAnonymous::resolveMaxCardinality(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::unordered_map<IndividualBranch*, std::vector<std::pair<std::string, size_t>>> indiv_indexes;
    bool valid = false;
    size_t nb_relations = 0;
    if(indiv->same_as_.empty())
    {
      if(expression->object_property_involved_ != nullptr)
        valid = resolveMaxCardinality(indiv->object_relations_.relations, expression->object_property_involved_, indiv_indexes[indiv], nb_relations, expression, used);
      else if(expression->data_property_involved_ != nullptr)
        valid = resolveMaxCardinality(indiv->data_relations_.relations, expression->data_property_involved_, indiv_indexes[indiv], nb_relations, expression, used);
    }
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
      {
        auto* same = indiv->same_as_[i].elem;
        if(expression->object_property_involved_ != nullptr)
          valid = resolveMaxCardinality(same->object_relations_.relations, expression->object_property_involved_, indiv_indexes[same], nb_relations, expression, used);
        else if(expression->data_property_involved_ != nullptr)
          valid = resolveMaxCardinality(same->data_relations_.relations, expression->data_property_involved_, indiv_indexes[same], nb_relations, expression, used);

        if(valid)
        {
          if((indiv_indexes[same].empty() == false) && (same != indiv))
            used.emplace_back(indiv->value() + "|sameAs|" + same->value(), indiv->same_as_.has_induced_inheritance_relations[i]);
        }
        else
          break;
      }
    }

    if(expression->restriction_type_ == RestrictionConstraintType_e::restriction_cardinality)
      valid = (nb_relations == expression->cardinality_value_);

    if(valid)
    {
      if(expression->object_property_involved_ != nullptr)
      {
        for(auto& indexes : indiv_indexes)
          for(auto& expl : indexes.second)
            used.emplace_back(indexes.first->value() + "|" + expl.first, indexes.first->object_relations_.has_induced_inheritance_relations[expl.second]);
      }
      else if(expression->data_property_involved_ != nullptr)
      {
        for(auto& indexes : indiv_indexes)
          for(auto& expl : indexes.second)
            used.emplace_back(indexes.first->value() + "|" + expl.first, indexes.first->data_relations_.has_induced_inheritance_relations[expl.second]);
      }
    }
    return valid; // used will be cleared by resolveRestriction if `valid` is false
  }

  bool ReasonerAnonymous::resolveMinCardinality(IndividualBranch* indiv, ClassExpression* expression, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::unordered_map<IndividualBranch*, std::vector<std::pair<std::string, size_t>>> indiv_indexes;
    bool valid = false;
    size_t nb_relations = 0;
    if(indiv->same_as_.empty())
    {
      if(expression->object_property_involved_ != nullptr)
        valid = resolveMinCardinality(indiv->object_relations_.relations, expression->object_property_involved_, indiv_indexes[indiv], nb_relations, expression, used);
      else if(expression->data_property_involved_ != nullptr)
        valid = resolveMinCardinality(indiv->data_relations_.relations, expression->data_property_involved_, indiv_indexes[indiv], nb_relations, expression, used);
    }
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
      {
        auto* same = indiv->same_as_[i].elem;
        if(expression->object_property_involved_ != nullptr)
          valid = resolveMinCardinality(same->object_relations_.relations, expression->object_property_involved_, indiv_indexes[same], nb_relations, expression, used);
        else if(expression->data_property_involved_ != nullptr)
          valid = resolveMinCardinality(same->data_relations_.relations, expression->data_property_involved_, indiv_indexes[same], nb_relations, expression, used);

        if((indiv_indexes[same].empty() == false) && (same != indiv))
          used.emplace_back(indiv->value() + "|sameAs|" + same->value(), indiv->same_as_.has_induced_inheritance_relations[i]);

        if(valid)
          break;
      }
    }

    if(valid)
    {
      if(expression->object_property_involved_ != nullptr)
      {
        for(auto& indexes : indiv_indexes)
          for(auto& expl : indexes.second)
            used.emplace_back(indexes.first->value() + "|" + expl.first, indexes.first->object_relations_.has_induced_inheritance_relations[expl.second]);
      }
      else if(expression->data_property_involved_ != nullptr)
      {
        for(auto& indexes : indiv_indexes)
          for(auto& expl : indexes.second)
            used.emplace_back(indexes.first->value() + "|" + expl.first, indexes.first->data_relations_.has_induced_inheritance_relations[expl.second]);
      }
    }
    return valid; // used will be cleared by resolveRestriction if `valid` is false
  }

  bool ReasonerAnonymous::checkClassesDisjointess(IndividualBranch* indiv, ClassBranch* class_equiv)
  {
    auto it = disjoints_cache_.find(class_equiv);
    std::unordered_set<ClassBranch*> disjoints;

    if(it != disjoints_cache_.end())
    {
      if(it->second.empty())
        return false;
      else
        disjoints = it->second;
    }
    else
    {
      ontology_->classes_.getDisjoint(class_equiv, disjoints);
      disjoints_cache_[class_equiv] = disjoints;
    }

    if(disjoints.empty() == false)
    {
      std::unordered_set<ClassBranch*> ups;
      ontology_->individuals_.getUpPtr(indiv, ups);
      return (ontology_->classes_.firstIntersection(ups, disjoints) != nullptr);
    }
    else
      return false;
  }

  bool ReasonerAnonymous::resolveDisjunctionTree(IndividualBranch* indiv, ClassExpression* ano_elem)
  {
    // works for class and class expression on object property range, not for data property
    // check the disjunctions between the indiv and the class elements in the ano_elem tree (not( Lidar and Sonar))
    if(ano_elem->type_ == ClassExpressionType_e::class_expression_intersection_of)
    {
      for(auto* elem : ano_elem->sub_elements_)
        if(resolveDisjunctionTree(indiv, elem) == false) // if false, one of the classes is not disjunctive, and thus the eq is not verif
          return false;
      return true;
    }
    else if(ano_elem->type_ == ClassExpressionType_e::class_expression_union_of)
    {
      for(auto* elem : ano_elem->sub_elements_)
        if(resolveDisjunctionTree(indiv, elem) == true) // if true, at least one in the or expression is disjoint, so eq is verif
          return true;
      return false;
    }
    else if(ano_elem->class_involved_ != nullptr)
      return checkClassesDisjointess(indiv, ano_elem->class_involved_); // actual check of disjointness

    return false;
  }

  std::string ReasonerAnonymous::getName()
  {
    return "reasoner anonymous";
  }

  std::string ReasonerAnonymous::getDescription()
  {
    return "This reasoner resolves the anonymous classes i.e the equivalence relations.";
  }

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerAnonymous, ontologenius::ReasonerInterface)