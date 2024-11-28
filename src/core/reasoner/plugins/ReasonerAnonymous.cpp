#include "ontologenius/core/reasoner/plugins/ReasonerAnonymous.h"

#include <cstddef>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
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
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_class(ontology_->class_graph_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_prop(ontology_->object_property_graph_.mutex_);
    std::vector<std::pair<std::string, InheritedRelationTriplets*>> used;

    for(auto* indiv : ontology_->individual_graph_.get())
    {
      if(first_run_ ||
         indiv->isUpdated() ||
         (indiv->flags_.find("equiv") != indiv->flags_.end()) ||
         indiv->hasUpdatedObjectRelation() ||
         indiv->hasUpdatedDataRelation() ||
         indiv->hasUpdatedInheritanceRelation())
      {
        bool has_active_equiv = false;

        // Loop over every classes which includes equivalence relations
        for(auto* anonymous_branch : ontology_->anonymous_graph_.get())
        {
          bool trees_evaluation_result = false;
          const bool is_already_a = std::any_of(indiv->is_a_.cbegin(), indiv->is_a_.cend(), [anonymous_branch](const auto& is_a) { return is_a.elem == anonymous_branch->class_equiv_; });

          if(is_already_a || checkClassesDisjointess(indiv, anonymous_branch->class_equiv_) == false)
          {
            // Loop over every equivalence relations corresponding to one class
            for(auto* anonymous_elem : anonymous_branch->ano_elems_)
            {
#ifdef DEBUG
              computeDebugUpdate(indiv, anonymous_elem);
#endif

              std::string equiv_flag = "equiv_" + anonymous_elem->ano_name;

              if((indiv->flags_.find(equiv_flag) != indiv->flags_.end()) || // already validated at least one member of an ano expression
                 ((indiv->isUpdated() == true) &&                           // indiv has been updated -> new individual
                  ((anonymous_elem->involves_class && indiv->is_a_.isUpdated()) ||
                   (anonymous_elem->involves_object_property && indiv->object_relations_.isUpdated()) ||
                   (anonymous_elem->involves_data_property && indiv->data_relations_.isUpdated()) ||
                   (anonymous_elem->involves_individual && indiv->same_as_.isUpdated()))))
              {
                bool tree_first_layer_result = true;
                bool current_tree_result = false;

                if(is_already_a == false)
                  tree_first_layer_result = resolveFirstLayer(indiv, anonymous_elem);
                has_active_equiv = has_active_equiv || tree_first_layer_result;

                if(tree_first_layer_result == true)
                {
                  indiv->flags_[equiv_flag] = {};
                  used.clear();
                  used.reserve(anonymous_branch->depth_);
                  current_tree_result = resolveTree(indiv, anonymous_elem, used);
                  trees_evaluation_result = trees_evaluation_result || current_tree_result;
                }
                else
                  indiv->flags_.erase(equiv_flag);

                if(has_active_equiv && current_tree_result)
                {
                  if(is_already_a == false) // the indiv is checked to still be of the same class so we can break out of the loop
                  {
                    addInferredInheritance(indiv, anonymous_branch, used);
                    nb_update++;
                    if(anonymous_branch->class_equiv_->isHidden() == false)
                    {
                      explanations_.emplace_back("[ADD]" + indiv->value() + "|isA|" + anonymous_branch->class_equiv_->value(),
                                                 "[ADD]" + indiv->is_a_.back().getExplanation());
                    }
                  }
                  // once we get a valid equivalence for a class, we break out of the loop
                  break;
                }
              }
            }
          }
          // used to remove inheritance in case an individual previously inferred does not check any of the expressions after updates
          if(trees_evaluation_result == false && anonymous_branch->ano_elems_.empty() == false && is_already_a)
          {
            indiv->nb_updates_++;
            anonymous_branch->class_equiv_->nb_updates_++;
            ontology_->individual_graph_.removeInheritage(indiv, anonymous_branch->class_equiv_, explanations_, true);
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

  void ReasonerAnonymous::addInferredInheritance(IndividualBranch* indiv, AnonymousClassBranch* anonymous_branch, std::vector<std::pair<std::string, InheritedRelationTriplets*>> used)
  {
    indiv->is_a_.emplaceBack(anonymous_branch->class_equiv_, 1.0, true); // adding the emplaceBack so that the is_a get in updated mode
    anonymous_branch->class_equiv_->individual_childs_.emplace_back(IndividualElement(indiv, 1.0, true));

    indiv->nb_updates_++;
    anonymous_branch->class_equiv_->nb_updates_++;

    for(auto& induced_vector : used)
    {
      indiv->is_a_.back().explanation.push_back(induced_vector.first);
      // check for nullptr because OneOf returns a (string, nullptr)
      if(induced_vector.second != nullptr)
      {
        if(induced_vector.second->exist(indiv, nullptr, anonymous_branch->class_equiv_) == false)
        {
          induced_vector.second->push(indiv, nullptr, anonymous_branch->class_equiv_);
          indiv->is_a_.relations.back().induced_traces.emplace_back(induced_vector.second);
        }
      }
    }
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
      ontology_->class_graph_.getDisjoint(class_equiv, disjoints);
      disjoints_cache_[class_equiv] = disjoints;
    }

    if(disjoints.empty() == false)
    {
      std::unordered_set<ClassBranch*> ups;
      ontology_->individual_graph_.getUpPtr(indiv, ups);
      return (ontology_->class_graph_.firstIntersection(ups, disjoints) != nullptr);
    }
    else
      return false;
  }

  bool ReasonerAnonymous::checkValue(IndividualBranch* indiv_from, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::string explanation;
    auto* indiv_range = ano_elem->individual_involved_;
    const size_t same_size = indiv_from->same_as_.size();

    for(size_t j = 0; j < same_size; j++)
    {
      if(indiv_from->same_as_[j].elem->get() == indiv_range->get())
      {
        explanation = indiv_from->value() + "|sameAs|" + indiv_range->value();
        used.emplace_back(explanation, indiv_from->same_as_.has_induced_inheritance_relations[j]);
        return true;
      }
    }

    if(indiv_from->get() == indiv_range->get())
      return true;

    return false;
  }

  bool ReasonerAnonymous::checkValue(LiteralNode* literal_from, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    (void)used;
    std::string explanation;
    auto* literal_range = ano_elem->card_.card_range_;

    if(literal_from->get() == literal_range->get())
      return true;

    return false;
  }

  bool ReasonerAnonymous::resolveFirstLayer(IndividualBranch* indiv, AnonymousClassElement* ano_elem)
  {
    if(ano_elem->logical_type_ == logical_and)
      for(auto* elem : ano_elem->sub_elements_)
        return resolveFirstLayer(indiv, elem);
    else if(ano_elem->logical_type_ == logical_or)
    {
      for(auto* elem : ano_elem->sub_elements_)
        if(resolveFirstLayer(indiv, elem) == true)
          return true;
      return false;
    }
    else if(ano_elem->logical_type_ == logical_not)
    {
      if(standard_mode_ == true)                                                         // OWA
        return resolveDisjunctionTreeFirstLayer(indiv, ano_elem->sub_elements_.front()); // need to make a version without the 'used' vector
      else                                                                               // CWA
        return !resolveFirstLayer(indiv, ano_elem->sub_elements_.front());
    }
    else
      return checkRestrictionFirstLayer(indiv, ano_elem);
    return false;
  }

  bool ReasonerAnonymous::resolveTree(LiteralNode* literal, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    if(ano_elem->logical_type_ == logical_and)
    {
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(resolveTree(literal, elem, used) == false)
        {
          used.clear();
          return false;
        }
      }
      return true;
    }
    else if(ano_elem->logical_type_ == logical_or)
    {
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(resolveTree(literal, elem, used))
          return true;
      }
      used.clear();
      return false;
    }
    else if(ano_elem->logical_type_ == logical_not)
      return !resolveTree(literal, ano_elem->sub_elements_.front(), used);
    else
      return checkTypeRestriction(literal, ano_elem, used);

    used.clear();
    return false;
  }

  bool ReasonerAnonymous::resolveTree(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    if(ano_elem->logical_type_ == logical_and)
    {
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(resolveTree(indiv, elem, used) == false)
        {
          used.clear();
          return false;
        }
      }
      return true;
    }
    else if(ano_elem->logical_type_ == logical_or)
    {
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(resolveTree(indiv, elem, used))
          return true;
      }
      used.clear();
      return false;
    }
    else if(ano_elem->logical_type_ == logical_not)
    {
      // do smth with used (maybe pass a tmp_used)
      if(standard_mode_ == true)
        return resolveDisjunctionTree(indiv, ano_elem->sub_elements_.front(), used);
      else
        return !resolveTree(indiv, ano_elem->sub_elements_.front(), used); // CWA
    }
    else
      return checkRestriction(indiv, ano_elem, used);

    return false;
  }

  bool ReasonerAnonymous::resolveDisjunctionTree(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    // works for class and class expression on object property range, not for data property
    // do we need to take into account the used, since disjointness isn't dynamic
    // check the disjunctions between the indiv and the class elements in the ano_elem tree (not( Lidar and Sonar))
    if(ano_elem->logical_type_ == logical_and)
    {
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(resolveDisjunctionTree(indiv, elem, used) == false) // if false, one of the classes is not disjunctive, and thus the eq is not verif
        {
          used.clear();
          return false;
        }
      }
      return true;
    }
    else if(ano_elem->logical_type_ == logical_or)
    {
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(resolveDisjunctionTree(indiv, elem, used) == true) // if true, at least one in the or expression is disjoint, so eq is verif
          return true;
      }
      used.clear();
      return false;
    }
    else if(ano_elem->class_involved_ != nullptr)
      return checkClassesDisjointess(indiv, ano_elem->class_involved_); // actual check of disjointness

    return false;
  }

  bool ReasonerAnonymous::resolveDisjunctionTreeFirstLayer(IndividualBranch* indiv, AnonymousClassElement* ano_elem)
  {
    // works for class and class expression on object property range, not for data property
    // do we need to take into account the used, since disjointness isn't dynamic
    // check the disjunctions between the indiv and the class elements in the ano_elem tree (not( Lidar and Sonar))
    if(ano_elem->logical_type_ == logical_and)
    {
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(resolveDisjunctionTreeFirstLayer(indiv, elem) == false)
          return false;
      }
      return true;
    }
    else if(ano_elem->logical_type_ == logical_or)
    {
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(resolveDisjunctionTreeFirstLayer(indiv, elem) == true)
          return true;
      }
      return false;
    }
    else if(ano_elem->class_involved_ != nullptr)
      return checkClassesDisjointess(indiv, ano_elem->class_involved_); // actual check of disjointness

    return false;
  }

  bool ReasonerAnonymous::checkRestrictionFirstLayer(IndividualBranch* indiv, AnonymousClassElement* ano_elem)
  {
    if(ano_elem->object_property_involved_ != nullptr)
    {
      if(indiv->same_as_.empty() == false)
      {
        for(auto& indiv_same : indiv->same_as_)
        {
          if(checkPropertyExistence(indiv_same.elem->object_relations_.relations, ano_elem))
            return true;
        }
      }
      return checkPropertyExistence(indiv->object_relations_.relations, ano_elem);
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      if(indiv->same_as_.empty() == false)
      {
        for(auto& indiv_same : indiv->same_as_)
        {
          if(checkPropertyExistence(indiv_same.elem->data_relations_.relations, ano_elem))
            return true;
        }
      }
      return checkPropertyExistence(indiv->data_relations_.relations, ano_elem);
    }
    else if(ano_elem->class_involved_ != nullptr)
      return (ontology_->individual_graph_.isA(indiv, ano_elem->class_involved_->get()));
    else if(ano_elem->oneof)
    {
      for(auto* indiv_elem : ano_elem->sub_elements_)
      {
        if(indiv->same_as_.empty() == false)
        {
          if(std::find_if(indiv->same_as_.begin(), indiv->same_as_.end(),
                          [elem = indiv_elem->individual_involved_](auto same) { return same == elem; }) != indiv->same_as_.end())
          {
            return true;
          }
        }
        if(indiv->get() == indiv_elem->individual_involved_->get())
          return true;
      }
    }
    return false;
  }

  bool ReasonerAnonymous::checkRestriction(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::string explanation;

    if(ano_elem->object_property_involved_ != nullptr || ano_elem->data_property_involved_ != nullptr)
    {
      if(indiv->same_as_.empty() == false)
      {
        const size_t same_size = indiv->same_as_.size();
        for(size_t i = 0; i < same_size; i++)
        {
          if(indiv->same_as_[i].elem->get() != indiv->get())
          {
            if(checkCard(indiv->same_as_[i].elem, ano_elem, used))
            {
              explanation = indiv->value() + "|sameAs|" + indiv->same_as_[i].elem->value();
              used.emplace_back(explanation, indiv->same_as_.has_induced_inheritance_relations[i]);
              return true;
            }
          }
        }
      }

      return checkCard(indiv, ano_elem, used);
    }
    else if(ano_elem->class_involved_ != nullptr)
      return checkTypeRestriction(indiv, ano_elem, used);
    else if(ano_elem->oneof)
    {
      std::string one_of;
      for(auto* elem : ano_elem->sub_elements_)
      {
        if(one_of.empty() == false)
          one_of += ", ";
        one_of += elem->individual_involved_->value();
        if(checkIndividualRestriction(indiv, elem, used) == true)
          explanation = indiv->value();
      }
      if(explanation.empty() == false)
      {
        explanation += "|isOneOf|(" + one_of + ")";
        used.emplace_back(explanation, nullptr);
        return true;
      }
    }

    used.clear();
    return false;
  }

  bool ReasonerAnonymous::checkTypeRestriction(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::string explanation;

    if(indiv->same_as_.empty() == false)
    {
      const size_t same_size = indiv->same_as_.size();
      for(size_t i = 0; i < same_size; i++)
      {
        if(indiv->same_as_[i].elem->get() != indiv->get())
        {
          const size_t is_a_size = indiv->same_as_[i].elem->is_a_.size();
          for(size_t j = 0; j < is_a_size; j++)
          {
            if(existInInheritance(indiv->same_as_[i].elem->is_a_[j].elem, ano_elem->class_involved_->get(), used))
            {
              explanation = indiv->same_as_[i].elem->value() + "|isA|" + ano_elem->class_involved_->value();
              used.emplace_back(explanation, indiv->same_as_[i].elem->is_a_.has_induced_inheritance_relations[j]);

              explanation = indiv->value() + "|sameAs|" + indiv->same_as_[i].elem->value();
              used.emplace_back(explanation, indiv->same_as_.has_induced_inheritance_relations[i]);
              return true;
            }
          }
        }
      }
    }
    else
    {
      const size_t is_a_size = indiv->is_a_.size();
      for(size_t i = 0; i < is_a_size; i++)
      {
        if(existInInheritance(indiv->is_a_[i].elem, ano_elem->class_involved_->get(), used))
        {
          explanation = indiv->value() + "|isA|" + ano_elem->class_involved_->value();
          used.emplace_back(explanation, indiv->is_a_.has_induced_inheritance_relations[i]);
          return true;
        }
      }
    }

    used.clear();
    return false;
  }

  bool ReasonerAnonymous::checkTypeRestriction(LiteralNode* literal, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    // To prevent warning of unused parameter for used
    (void)used;
    return (literal->type_ == ano_elem->card_.card_range_->type_);
  }

  bool ReasonerAnonymous::checkIndividualRestriction(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::string explanation;

    if(indiv->same_as_.empty() == false)
    {
      const size_t same_size = indiv->same_as_.size();
      for(size_t i = 0; i < same_size; i++)
      {
        if(indiv->same_as_[i].elem->get() != indiv->get())
        {
          if(indiv->same_as_[i].elem->get() == ano_elem->individual_involved_->get())
          {
            explanation = indiv->value() + "|sameAs|" + ano_elem->individual_involved_->value();
            used.emplace_back(explanation, indiv->same_as_.has_induced_inheritance_relations[i]);
            return true;
          }
        }
      }
    }

    return (indiv->get() == ano_elem->individual_involved_->get());
  }

  bool ReasonerAnonymous::checkCard(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    switch(ano_elem->card_.card_type_)
    {
    case CardType_e::cardinality_some:
      return checkSomeCard(indiv, ano_elem, used);
    case CardType_e::cardinality_min:
      if(standard_mode_ == true)
        return false;
      else
        return checkMinCard(indiv, ano_elem, used);
    case CardType_e::cardinality_max:
      if(standard_mode_ == true)
        return false;
      else
        return checkMaxCard(indiv, ano_elem, used);
    case CardType_e::cardinality_exactly:
      if(standard_mode_ == true)
        return false;
      else
        return checkExactlyCard(indiv, ano_elem, used);
    case CardType_e::cardinality_only:
      if(standard_mode_ == true)
        return false;
      else
        return checkOnlyCard(indiv, ano_elem, used);
    case CardType_e::cardinality_value:
      return checkValueCard(indiv, ano_elem, used);
    default:
      Display::error("Cardinality type outside of [min, max, exactly, only, value, some]");
      return false;
    }
  }

  bool ReasonerAnonymous::checkMinCard(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::vector<std::pair<std::string, size_t>> indexes;

    if(ano_elem->object_property_involved_ != nullptr)
    {
      indexes = checkMinCard(indiv->object_relations_.relations, ano_elem, used);
      if(indexes.empty() == false)
      {
        for(auto& index : indexes)
          used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      indexes = checkMinCard(indiv->data_relations_.relations, ano_elem, used);
      if(indexes.empty() == false)
      {
        for(auto& index : indexes)
          used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    used.clear();
    return false;
  }

  bool ReasonerAnonymous::checkMaxCard(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::vector<std::pair<std::string, size_t>> indexes;

    if(ano_elem->object_property_involved_ != nullptr)
    {
      indexes = checkMaxCard(indiv->object_relations_.relations, ano_elem, used);
      if(indexes.empty() == false)
      {
        for(auto& index : indexes)
          used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      indexes = checkMaxCard(indiv->data_relations_.relations, ano_elem, used);
      if(indexes.empty() == false)
      {
        for(auto& index : indexes)
          used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    used.clear();
    return false;
  }

  bool ReasonerAnonymous::checkExactlyCard(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::vector<std::pair<std::string, size_t>> indexes;

    if(ano_elem->object_property_involved_ != nullptr)
    {
      indexes = checkExactlyCard(indiv->object_relations_.relations, ano_elem, used);
      if(indexes.empty() == false)
      {
        for(auto& index : indexes)
          used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      indexes = checkExactlyCard(indiv->data_relations_.relations, ano_elem, used);
      if(indexes.empty() == false)
      {
        for(auto& index : indexes)
          used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    used.clear();
    return false;
  }

  bool ReasonerAnonymous::checkOnlyCard(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::vector<std::pair<std::string, size_t>> indexes;

    if(ano_elem->object_property_involved_ != nullptr)
    {
      indexes = checkOnlyCard(indiv->object_relations_.relations, ano_elem, used);
      if(indexes.empty() == false)
      {
        for(auto& index : indexes)
          used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      indexes = checkOnlyCard(indiv->data_relations_.relations, ano_elem, used);
      if(indexes.empty() == false)
      {
        for(auto& index : indexes)
          used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    used.clear();
    return false;
  }

  bool ReasonerAnonymous::checkSomeCard(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::pair<std::string, int> index;

    if(ano_elem->object_property_involved_ != nullptr)
    {
      index = checkSomeCard(indiv->object_relations_.relations, ano_elem, used);
      if(index.second != -1)
      {
        used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      index = checkSomeCard(indiv->data_relations_.relations, ano_elem, used);
      if(index.second != -1)
      {
        used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    used.clear();
    return false;
  }

  bool ReasonerAnonymous::checkValueCard(IndividualBranch* indiv, AnonymousClassElement* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
  {
    std::pair<std::string, int> index;
    std::string explanation;

    if(ano_elem->object_property_involved_ != nullptr)
    {
      index = checkValueCard(indiv->object_relations_.relations, ano_elem, used);
      if(index.second != -1)
      {
        used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      index = checkValueCard(indiv->data_relations_.relations, ano_elem, used);
      if(index.second != -1)
      {
        used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
        return true;
      }
    }

    used.clear();
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