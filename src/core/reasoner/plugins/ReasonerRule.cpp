#include "ontologenius/core/reasoner/plugins/ReasonerRule.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  ReasonerRule::ReasonerRule() : standard_mode_(false)
  {}

  void ReasonerRule::setParameter(const std::string& name, const std::string& value)
  {
    if(name == "standard_mode" && value == "true")
      standard_mode_ = true;
  }

  void ReasonerRule::postReason()
  {
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individuals_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_class(ontology_->classes_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_obj_prop(ontology_->object_properties_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_data_prop(ontology_->data_properties_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_ano(ontology_->anonymous_classes_.mutex_);

    for(auto* rule_branch : ontology_->rules_.get())
    {
      std::vector<index_t> empty_accu(rule_branch->variables_.size(), index_t()); // need to initialize each index at 0

      std::vector<RuleResult_t> results_resolve = resolveBody(rule_branch, rule_branch->rule_body_, empty_accu);

      for(auto& solution : results_resolve) // resolve the consequent for each found solution
        resolveHead(rule_branch->rule_head_, solution, rule_branch);
    }
  }

  void ReasonerRule::resolveHead(const std::vector<RuleTriplet_t>& atoms, const RuleResult_t& solution, RuleBranch* rule)
  {
    for(const auto& atom : atoms)
    {
      switch(atom.atom_type_)
      {
      case rule_atom_class:
        addInferredClassAtom(atom, solution, rule);
        break;
      case rule_atom_object:
        addInferredObjectAtom(atom, solution, rule);
        break;
      case rule_atom_data:
        addInferredDataAtom(atom, solution, rule);
        break;
      case rule_atom_builtin:
        break;

      default:
        break;
      }
    }
  }

  bool ReasonerRule::checkClassesDisjointess(IndividualBranch* indiv, ClassBranch* class_equiv) // copy from ReasonerAnonymous
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

  void ReasonerRule::addInferredClassAtom(const RuleTriplet_t& triplet, const RuleResult_t& solution, RuleBranch* rule) // maybe not const since we mark the updates
  {
    IndividualBranch* involved_indiv = nullptr;

    const auto& subject = triplet.arguments[0];

    if(subject.is_variable && (solution.assigned_result.at(subject.variable_id) != 0))
      involved_indiv = ontology_->individuals_.findBranch(solution.assigned_result.at(subject.variable_id));
    else
      involved_indiv = subject.indiv_value;

    const bool is_already_a = std::any_of(involved_indiv->is_a_.cbegin(), involved_indiv->is_a_.cend(), [triplet](const auto& is_a) { return is_a.elem == triplet.class_predicate; });

    if(involved_indiv != nullptr)
    {
      if((is_already_a == false) && (checkClassesDisjointess(involved_indiv, triplet.class_predicate) == false))
      {
        ontology_->individuals_.addClassAssertion(involved_indiv, triplet.class_predicate, 1.0, true);

        involved_indiv->nb_updates_++;
        triplet.class_predicate->nb_updates_++;

        // insert all explanations since they all are the reason why it has been inferred
        involved_indiv->is_a_.back().explanation.insert(involved_indiv->is_a_.back().explanation.end(),
                                                        solution.explanations.cbegin(),
                                                        solution.explanations.cend());
        // add the pointer to the rule_branch used to make that inference
        involved_indiv->is_a_.back().used_rule = rule;

        for(const auto& used : solution.triplets_used)
        {
          auto* inheritance_triplet = used.getInheritance();
          if(inheritance_triplet->exist(involved_indiv, nullptr, triplet.class_predicate) == false)
          {
            inheritance_triplet->push(involved_indiv, nullptr, triplet.class_predicate);
            involved_indiv->is_a_.relations.back().induced_traces.emplace_back(inheritance_triplet);
          }
        }

        nb_update++;
        explanations_.emplace_back("[ADD]" + involved_indiv->value() + "|isA|" + triplet.class_predicate->value(),
                                   "[ADD]" + involved_indiv->is_a_.back().getExplanation());
      }
    }
  }

  void ReasonerRule::addInferredObjectAtom(const RuleTriplet_t& triplet, const RuleResult_t& solution, RuleBranch* rule)
  {
    IndividualBranch* involved_indiv_from = nullptr;
    IndividualBranch* involved_indiv_on = nullptr;

    const auto& subject = triplet.arguments[0];
    const auto& object = triplet.arguments[1];

    if(subject.is_variable && (solution.assigned_result.at(subject.variable_id) != 0))
      involved_indiv_from = ontology_->individuals_.findBranch(solution.assigned_result[subject.variable_id]);
    else
      involved_indiv_from = subject.indiv_value;
    if(object.is_variable && (solution.assigned_result.at(object.variable_id) != 0))
      involved_indiv_on = ontology_->individuals_.findBranch(solution.assigned_result[object.variable_id]);
    else
      involved_indiv_on = object.indiv_value;

    if(involved_indiv_from != nullptr && involved_indiv_on != nullptr)
    {
      if(ontology_->individuals_.relationExists(involved_indiv_from, triplet.object_predicate, involved_indiv_on) == false)
      {
        int relation_index = ontology_->individuals_.addRelation(involved_indiv_from, triplet.object_predicate, involved_indiv_on, 1.0, true, false);
        involved_indiv_from->nb_updates_++;

        involved_indiv_from->object_relations_[relation_index].explanation.insert(involved_indiv_from->object_relations_[relation_index].explanation.end(),
                                                                                  solution.explanations.cbegin(),
                                                                                  solution.explanations.cend());
        involved_indiv_from->object_relations_[relation_index].used_rule = rule;

        for(const auto& used : solution.triplets_used)
        {
          auto* object_triplet = used.getObject();
          if(object_triplet->exist(involved_indiv_from, triplet.object_predicate, involved_indiv_on) == false)
          {
            object_triplet->push(involved_indiv_from, triplet.object_predicate, involved_indiv_on);
            involved_indiv_from->object_relations_.relations[relation_index].induced_traces.emplace_back(object_triplet); // need to add every X_triplet used to induced_traces
          }
        }

        nb_update++;
        explanations_.emplace_back("[ADD]" + involved_indiv_from->value() + "|" + triplet.object_predicate->value() + "|" + involved_indiv_on->value(),
                                   "[ADD]" + involved_indiv_from->object_relations_[relation_index].getExplanation());
      }
    }
  }

  void ReasonerRule::addInferredDataAtom(const RuleTriplet_t& triplet, const RuleResult_t& solution, RuleBranch* rule)
  {
    IndividualBranch* involved_indiv_from = nullptr;
    LiteralNode* involved_literal_on = nullptr;

    const auto& subject = triplet.arguments[0];
    const auto& object = triplet.arguments[1];

    if(subject.is_variable && (solution.assigned_result.at(subject.variable_id) != 0))
      involved_indiv_from = ontology_->individuals_.findBranch(solution.assigned_result[subject.variable_id]);
    else
      involved_indiv_from = subject.indiv_value;
    if(object.is_variable && (solution.assigned_result.at(object.variable_id) != 0))
      involved_literal_on = ontology_->literals_.find(solution.assigned_result.at(object.variable_id));
    else
      involved_literal_on = object.datatype_value;

    if(involved_indiv_from != nullptr && involved_literal_on != nullptr)
    {
      if(ontology_->individuals_.relationExists(involved_indiv_from, triplet.data_predicate, involved_literal_on) == false)
      {
        int relation_index = ontology_->individuals_.addRelation(involved_indiv_from, triplet.data_predicate, involved_literal_on, 1.0, true, false);
        involved_indiv_from->nb_updates_++;

        involved_indiv_from->data_relations_[relation_index].explanation.insert(involved_indiv_from->data_relations_[relation_index].explanation.end(),
                                                                                solution.explanations.cbegin(),
                                                                                solution.explanations.cend());
        involved_indiv_from->data_relations_[relation_index].used_rule = rule;

        for(const auto& used : solution.triplets_used)
        {
          auto* data_triplet = used.getData();
          if(data_triplet->exist(involved_indiv_from, triplet.data_predicate, involved_literal_on) == false)
          {
            data_triplet->push(involved_indiv_from, triplet.data_predicate, involved_literal_on); // write into has_induced_data_relation for the newly asserted relation
            involved_indiv_from->data_relations_[relation_index].induced_traces.emplace_back(data_triplet);
          }
        }
        nb_update++;
        explanations_.emplace_back("[ADD]" + involved_indiv_from->value() + "|" + triplet.data_predicate->value() + "|" + involved_literal_on->value(),
                                   "[ADD]" + involved_indiv_from->data_relations_[relation_index].getExplanation());
      }
    }
  }

  std::vector<RuleResult_t> ReasonerRule::resolveBody(RuleBranch* rule_branch, std::vector<RuleTriplet_t>& atoms, std::vector<index_t>& accu)
  {
    std::vector<IndivResult_t> values;
    int64_t var_index = 0;
    resolveAtom(atoms.front(), accu, var_index, values); // returns the individuals matching the triplet (with explanations and used_relations)

    if(values.empty() == true)
      return {};

    if(atoms.size() > 1) // means that we haven't reached the end of the rule antecedents
    {
      std::vector<RuleTriplet_t> new_atoms(atoms.begin() + 1, atoms.end()); // move on to the next atom

      std::vector<RuleResult_t> res;
      res.reserve(values.size());          // values.size() -> number of new branch to explore
      std::vector<index_t> new_accu(accu); // create a copy of the state of the accu (instantiated variables so far)

      for(auto& value : values)
      {
        if(var_index != -1) // if it effectively refer to a variable
        {
          if(accu.at(var_index) != 0) // the variable has already been assigned with a value
          {
            if((accu.at(var_index) > 0) && (accu.at(var_index) != value.indiv->get())) // if the previously assigned indiv value and the new one are different, we continue
              continue;
            else if((accu.at(var_index) < 0) && (accu.at(var_index) != value.literal->get())) // if the previously assigned literal value and the new one are different, we continue
              continue;
          }
          else
          {
            if(value.indiv != nullptr)
              new_accu.at(var_index) = value.indiv->get();
            else if(value.literal != nullptr)
              new_accu.at(var_index) = value.literal->get();
            else
              std::cout << "No value was returned" << std::endl;
          }
        }

        std::vector<RuleResult_t> local_res = resolveBody(rule_branch, new_atoms, new_accu); // new solutions updated with the new_accu

        if(local_res.empty() == false)
        {
          for(auto& lr : local_res)
          {
            lr.insertResult(value, var_index);
            res.push_back(std::move(lr));
          }
        }
      }
      return res;
    }
    else // we reached the end of an exploration, we backpropagate each instantiated variable into the resulting vector
    {
      std::vector<RuleResult_t> res(values.size(), RuleResult_t(rule_branch->variables_.size())); // we have one RuleResult_t per end of exploration
      size_t cpt = 0;

      for(auto& value : values)
        res[cpt++].insertResult(value, var_index);

      return res;
    }
  }

  void ReasonerRule::resolveAtom(RuleTriplet_t& triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    switch(triplet.atom_type_)
    {
    case rule_atom_class:
      resolveClassAtom(triplet, accu, var_index, values);
      break;
    case rule_atom_object:
      resolveObjectAtom(triplet, accu, var_index, values);
      break;
    case rule_atom_data:
      resolveDataAtom(triplet, accu, var_index, values);
      break;
    case rule_atom_builtin:
      resolveBuiltinAtom(triplet, accu, var_index, values);
      break;
    default:
      break;
    }
  }

  void ReasonerRule::resolveClassAtom(const RuleTriplet_t& triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    const auto& subject = triplet.arguments[0];
    var_index = subject.variable_id;
    if(subject.is_variable == true)
    {
      if(accu.at(var_index) == 0) // has no previous value
        getType(triplet.class_predicate, values);
      else
      {
        IndividualBranch* involved_indiv = ontology_->individuals_.findBranch(accu.at(var_index));
        auto res = isA(involved_indiv, triplet.class_predicate);
        if(res.empty() == false)
          values.emplace_back(std::move(res));
      }
    }
    else
    {
      // atom is not variable, so we check the individual in the triplet
      auto res = isA(subject.indiv_value, triplet.class_predicate);
      if(res.empty() == false)
        values.emplace_back(std::move(res));
    }
  }

  void ReasonerRule::getType(ClassBranch* class_selector, std::vector<IndivResult_t>& res, const IndivResult_t& prev, ClassBranch* main_class_predicate)
  {
    if(main_class_predicate == nullptr) // set the main_class_predicate only for the first function call
      main_class_predicate = class_selector;

    for(const auto& child : class_selector->individual_childs_) // Inheritance individual/class
    {
      auto local_res = prev;
      IndividualBranch* indiv = child.elem;
      local_res.indiv = indiv;
      for(size_t i = 0; i < indiv->is_a_.size(); i++)
      {
        if(indiv->is_a_[i] == class_selector)
        {
          if(class_selector == main_class_predicate)
            constructResult(indiv->value(), indiv->is_a_, i, local_res, true);
          else
          {
            std::string explanation = indiv->value() + "|isA|" + main_class_predicate->value(); // main explanation
            local_res.explanations.emplace_back(explanation);
            constructResult(indiv->value(), indiv->is_a_, i, local_res, false);
          }

          break;
        }
      }

      res.emplace_back(std::move(local_res));
    }

    for(const auto& child : class_selector->childs_) // Inheritance class/class
    {
      auto local_res = prev;
      ClassBranch* child_ptr = child.elem;
      for(size_t i = 0; i < child_ptr->mothers_.size(); i++)
      {
        if(child_ptr->mothers_[i] == class_selector)
        {
          constructResult(child_ptr->value(), child_ptr->mothers_, i, local_res, false);
          break;
        }
      }

      getType(child_ptr, res, local_res, main_class_predicate);
    }
  }

  void ReasonerRule::constructResult(const std::string& concept_name,
                                     const RelationsWithInductions<IndividualElement>& relation,
                                     size_t index, IndivResult_t& res)
  {
    std::string explanation = concept_name + "|sameAs|" + relation.at(index).elem->value();
    res.explanations.emplace_back(explanation);

    res.used_triplets.emplace_back(relation.has_induced_inheritance_relations[index],
                                   relation.has_induced_object_relations[index],
                                   relation.has_induced_data_relations[index]);
  }

  IndivResult_t ReasonerRule::isA(IndividualBranch* indiv, ClassBranch* class_selector)
  {
    IndivResult_t res;
    if(indiv->same_as_.empty())
    {
      if(isA(indiv->value(), class_selector, indiv->is_a_, res))
        res.indiv = indiv;
    }
    else
    {
      for(size_t i = 0; i < indiv->same_as_.size(); i++)
        if(isA(indiv->same_as_.at(i).elem->value(), class_selector, indiv->same_as_.at(i).elem->is_a_, res))
        {
          res.indiv = indiv;
          if(indiv != indiv->same_as_[i].elem)
            constructResult(indiv->value(), indiv->same_as_, i, res);
          break;
        }
    }
    return res;
  }

  void ReasonerRule::resolveObjectAtom(RuleTriplet_t& triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    auto& subject = triplet.arguments[0];
    auto& object = triplet.arguments[1];
    if(!subject.is_variable && !object.is_variable)
    {
      values = getOnObject(triplet, object.indiv_value->get());
      var_index = -1; // not refered to a variable
    }
    else if(subject.is_variable && !object.is_variable)
    {
      var_index = subject.variable_id;
      if(accu.at(var_index) == 0)
        values = getFromObject(triplet);
      else
      {
        subject.indiv_value = ontology_->individuals_.findBranch(accu.at(var_index));
        var_index = object.variable_id;
        values = getOnObject(triplet, object.indiv_value->get());
      }
    }
    else if(!subject.is_variable && object.is_variable)
    {
      var_index = object.variable_id;
      values = getOnObject(triplet, accu.at(var_index));
    }
    else if(subject.is_variable && object.is_variable)
    {
      var_index = subject.variable_id;
      if(accu.at(var_index) != 0)
      {
        subject.indiv_value = ontology_->individuals_.findBranch(accu.at(var_index));
        var_index = object.variable_id;
        values = getOnObject(triplet, accu.at(var_index));
      }
      else if(accu[object.variable_id] != 0)
      {
        object.indiv_value = ontology_->individuals_.findBranch(accu[object.variable_id]);
        values = getFromObject(triplet);
      }
      else
        std::cout << "no variable bounded to any of the variable fields" << std::endl;
    }
  }

  std::vector<IndivResult_t> ReasonerRule::getFromObject(const RuleTriplet_t& triplet)
  {
    std::vector<IndivResult_t> res;
    const auto& object = triplet.arguments[1];

    if(object.indiv_value->same_as_.empty())
    {
      for(auto& indiv : ontology_->individuals_.all_branchs_)
        for(size_t i = 0; i < indiv->object_relations_.size(); i++)
        {
          const auto& relations = indiv->object_relations_;
          if(relations.at(i).second == object.indiv_value)
          {
            IndivResult_t local_res;
            if((relations.at(i).first == triplet.object_predicate) ||
               isA(relations.at(i).first->value(), triplet.object_predicate, relations.at(i).first->mothers_, local_res))
            {
              local_res.indiv = indiv;
              constructResult(indiv->value(), relations, i, local_res);
              res.emplace_back(std::move(local_res));
            }
          }
        }
    }
    else
    {
      for(auto& indiv : ontology_->individuals_.all_branchs_)
        for(size_t i = 0; i < indiv->object_relations_.size(); i++)
        {
          for(size_t j = 0; j < object.indiv_value->same_as_.size(); j++)
          {
            const auto& relations = indiv->object_relations_;
            if(relations.at(i).second == object.indiv_value->same_as_[j].elem)
            {
              IndivResult_t local_res;
              if((relations.at(i).first == triplet.object_predicate) ||
                 isA(relations.at(i).first->value(), triplet.object_predicate, relations.at(i).first->mothers_, local_res))
              {
                local_res.indiv = indiv;
                constructResult(object.indiv_value->same_as_[i].elem->value(), relations, i, local_res);

                if(object.indiv_value != object.indiv_value->same_as_[i].elem)
                  constructResult(object.indiv_value->value(), object.indiv_value->same_as_, j, local_res);

                res.emplace_back(std::move(local_res));
                break;
              }
            }
          }
        }
    }

    return res;
  }

  std::vector<IndivResult_t> ReasonerRule::getOnObject(const RuleTriplet_t& triplet, index_t selector)
  {
    std::vector<IndivResult_t> res;
    IndividualBranch* indiv = triplet.arguments[0].indiv_value;
    IndividualBranch* selector_ptr = nullptr;
    if(selector != 0)
      selector_ptr = ontology_->individuals_.findBranch(selector);

    if(indiv->same_as_.empty())
    {
      const auto& relations = indiv->object_relations_;
      for(size_t i = 0; i < relations.size(); i++)
      {
        IndivResult_t local_res;
        if((relations.at(i).first == triplet.object_predicate) ||
           isA(relations.at(i).first->value(), triplet.object_predicate, relations.at(i).first->mothers_, local_res))
        {
          if(selector == 0)
          {
            local_res.indiv = relations.at(i).second;
            constructResult(indiv->value(), relations, i, local_res);
            res.emplace_back(std::move(local_res));
          }
          else if(selector_ptr->same_as_.empty())
          {
            if(selector_ptr == relations.at(i).second)
            {
              local_res.indiv = relations.at(i).second;
              constructResult(indiv->value(), relations, i, local_res);
              res.emplace_back(std::move(local_res));
            }
          }
          else
          {
            for(size_t j = 0; j < selector_ptr->same_as_.size(); j++)
              if(relations.at(i).second == selector_ptr->same_as_[j].elem)
              {
                IndivResult_t local_res;
                local_res.indiv = relations.at(i).second;

                if(selector_ptr != selector_ptr->same_as_[j].elem)
                  constructResult(selector_ptr->value(), selector_ptr->same_as_, j, local_res);

                constructResult(indiv->value(), relations, i, local_res);
                res.emplace_back(std::move(local_res));
              }
          }
        }
      }
    }
    else
    {
      for(size_t s = 0; s < indiv->same_as_.size(); s++)
      {
        const IndividualBranch* same = indiv->same_as_[s].elem;
        const auto& relations = same->object_relations_;
        for(size_t i = 0; i < relations.size(); i++)
        {
          IndivResult_t local_res;
          if((relations.at(i).first == triplet.object_predicate) ||
             isA(relations.at(i).first->value(), triplet.object_predicate, relations.at(i).first->mothers_, local_res))
          {
            if(selector == 0)
            {
              local_res.indiv = relations.at(i).second;

              if(same != indiv)
                constructResult(same->value(), same->same_as_, s, local_res);

              constructResult(same->value(), relations, i, local_res);
              res.emplace_back(std::move(local_res));
            }
            else if(selector_ptr->same_as_.empty())
            {
              if(selector_ptr == relations.at(i).second)
              {
                local_res.indiv = relations.at(i).second;

                if(same != indiv)
                  constructResult(same->value(), same->same_as_, s, local_res);

                constructResult(same->value(), relations, i, local_res);
                res.emplace_back(std::move(local_res));
              }
            }
            else
            {
              for(size_t j = 0; j < selector_ptr->same_as_.size(); j++)
                if(relations.at(i).second == selector_ptr->same_as_[j].elem)
                {
                  IndivResult_t local_res;
                  local_res.indiv = relations.at(i).second;

                  if(same != indiv)
                    constructResult(same->value(), same->same_as_, s, local_res);

                  if(selector_ptr != selector_ptr->same_as_[j].elem)
                    constructResult(selector_ptr->value(), selector_ptr->same_as_, j, local_res);

                  constructResult(same->value(), relations, i, local_res);
                  res.emplace_back(std::move(local_res));
                }
            }
          }
        }
      }
    }

    return res;
  }

  void ReasonerRule::resolveDataAtom(RuleTriplet_t& triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    auto& subject = triplet.arguments[0];
    auto& object = triplet.arguments[1];
    if(!subject.is_variable && !object.is_variable)
    {
      values = getOnData(triplet, object.datatype_value->get());
      var_index = -1; // not refered to a variable
    }
    else if(subject.is_variable && !object.is_variable)
    {
      var_index = subject.variable_id;
      if(accu.at(var_index) == 0)
        values = getFromData(triplet);
      else
      {
        subject.indiv_value = ontology_->individuals_.findBranch(accu.at(var_index));
        var_index = object.variable_id;
        values = getOnData(triplet, object.datatype_value->get());
      }
    }
    else if(!subject.is_variable && object.is_variable)
    {
      var_index = object.variable_id;
      values = getOnData(triplet, accu.at(var_index));
    }
    else if(subject.is_variable && object.is_variable)
    {
      var_index = subject.variable_id;
      if(accu.at(var_index) != 0)
      {
        subject.indiv_value = ontology_->individuals_.findBranch(accu.at(var_index));
        var_index = object.variable_id;
        values = getOnData(triplet, accu.at(var_index));
      }
      else if(accu[object.variable_id] != 0)
      {
        object.datatype_value = ontology_->literals_.find(accu[object.variable_id]);
        values = getFromData(triplet);
      }
      else
        std::cout << "no variable bounded to any of the variable fields" << std::endl;
    }
  }

  std::vector<IndivResult_t> ReasonerRule::getFromData(const RuleTriplet_t& triplet)
  {
    std::vector<IndivResult_t> res;

    for(auto& indiv : ontology_->individuals_.all_branchs_)
      for(size_t i = 0; i < indiv->data_relations_.size(); i++)
      {
        const auto& relations = indiv->data_relations_;
        if(relations.at(i).second == triplet.arguments[1].datatype_value)
        {
          IndivResult_t local_res;
          if((relations.at(i).first == triplet.data_predicate) ||
             isA(relations.at(i).first->value(), triplet.data_predicate, relations.at(i).first->mothers_, local_res))
          {
            local_res.indiv = indiv;
            constructResult(indiv->value(), relations, i, local_res);
            res.emplace_back(std::move(local_res));
          }
        }
      }

    return res;
  }

  std::vector<IndivResult_t> ReasonerRule::getOnData(const RuleTriplet_t& triplet, index_t selector)
  {
    std::vector<IndivResult_t> res;
    IndividualBranch* indiv = triplet.arguments[0].indiv_value;

    if(indiv->same_as_.empty())
    {
      const auto& relations = indiv->data_relations_;
      for(size_t i = 0; i < relations.size(); i++)
      {
        IndivResult_t local_res;
        if((relations.at(i).first == triplet.data_predicate) ||
           isA(relations.at(i).first->value(), triplet.data_predicate, relations.at(i).first->mothers_, local_res))
        {
          if((selector == 0) || (selector == relations.at(i).second->get()))
          {
            local_res.literal = relations.at(i).second;
            constructResult(indiv->value(), relations, i, local_res);
            res.emplace_back(std::move(local_res));
          }
        }
      }
    }
    else
    {
      for(size_t s = 0; s < indiv->same_as_.size(); s++)
      {
        const IndividualBranch* same = indiv->same_as_[s].elem;
        const auto& relations = same->data_relations_;
        for(size_t i = 0; i < relations.size(); i++)
        {
          IndivResult_t local_res;
          if((relations.at(i).first == triplet.data_predicate) ||
             isA(relations.at(i).first->value(), triplet.data_predicate, relations.at(i).first->mothers_, local_res))
          {
            if((selector == 0) || (selector == relations.at(i).second->get()))
            {
              local_res.literal = relations.at(i).second;

              if(same != indiv)
                constructResult(same->value(), same->same_as_, s, local_res);

              constructResult(same->value(), relations, i, local_res);
              res.emplace_back(std::move(local_res));
            }
          }
        }
      }
    }

    return res;
  }

  void ReasonerRule::resolveBuiltinAtom(RuleTriplet_t& triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    LiteralNode* subject_ptr = nullptr;
    LiteralNode* object_ptr = nullptr;
    IndivResult_t res;

    auto& subject = triplet.arguments[0];
    auto& object = triplet.arguments[1];
    var_index = subject.variable_id;

    if((subject.variable_id != -1) && (accu.at(subject.variable_id) != 0))
      subject_ptr = ontology_->literals_.find(accu.at(subject.variable_id));
    else if(subject.datatype_value != nullptr)
      subject_ptr = subject.datatype_value;
    else
    {
      std::cout << "No value for arg1 of the builtin :" << triplet.builtinToString() << std::endl;
      return;
    }

    if((object.variable_id != -1) && (accu.at(object.variable_id) != 0))
      object_ptr = ontology_->literals_.find(accu.at(object.variable_id));
    else if(object.datatype_value != nullptr)
      object_ptr = object.datatype_value;
    else
    {
      std::cout << "No value for arg2 of the builtin :" << triplet.builtinToString() << std::endl;
      return;
    }

    if((subject_ptr != nullptr) && (object_ptr != nullptr))
    {
      if((subject_ptr->type_->value() == "string") && (object_ptr->type_ == subject_ptr->type_) && // both are strings
         (resolveStringBuiltinAtom(triplet.builtin, subject_ptr, object_ptr) == true))
      {
        res.literal = subject_ptr;
        std::string explanation = triplet.builtinToString() + "(" + subject_ptr->value() + "," + object_ptr->value();
        res.explanations.emplace_back(explanation);
        values.emplace_back(std::move(res));
      }
      else if(resolveNumericalBuiltinAtom(triplet.builtin, subject_ptr, object_ptr) == true)
      {
        res.literal = subject_ptr;
        std::string explanation = triplet.builtinToString() + "(" + subject_ptr->value() + "," + object_ptr->value();
        res.explanations.emplace_back(explanation);
        values.emplace_back(std::move(res));
      }
    }
  }

  bool ReasonerRule::resolveNumericalBuiltinAtom(RuleBuiltinType_e builtin_type, LiteralNode* subject, LiteralNode* object)
  {
    try
    {
      const double subject_cast = stod(subject->data());
      const double object_cast = stod(object->data());

      switch(builtin_type)
      {
      case builtin_greater_than:
        return subject_cast > object_cast;
      case builtin_greater_than_or_equal:
        return subject_cast >= object_cast;
      case builtin_less_than:
        return subject_cast < object_cast;
      case builtin_less_than_or_equal:
        return subject_cast <= object_cast;
      case builtin_equal:
        return subject_cast == object_cast;
      case builtin_not_equal:
        return subject_cast != object_cast;
      default:
        std::cout << "Unsupported builtin type : " << builtin_type << "for numerical arguments" << std::endl;
        return false;
      }
    }
    catch(std::invalid_argument const& ex)
    {
      std::cout << "cannot convert either arg1 or arg2 to double" << std::endl;
      return false;
    }
  }

  bool ReasonerRule::resolveStringBuiltinAtom(RuleBuiltinType_e builtin_type, LiteralNode* subject, LiteralNode* object)
  {
    switch(builtin_type)
    {
    case builtin_equal:
      return subject == object;
    case builtin_not_equal:
      return subject != object;
    default:
      std::cout << "Unsupported builtin type : " << builtin_type << "for string arguments" << std::endl;
      return false;
    }
  }

  std::string ReasonerRule::getName()
  {
    return "reasoner rule";
  }

  std::string ReasonerRule::getDescription()
  {
    return "This is a reasoner for SWRL rules.";
  }

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerRule, ontologenius::ReasonerInterface)