#include "ontologenius/core/reasoner/plugins/ReasonerRule.h"

#include <cstddef>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"

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
    const std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_class(ontology_->class_graph_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_obj_prop(ontology_->object_property_graph_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_data_prop(ontology_->data_property_graph_.mutex_);
    const std::shared_lock<std::shared_timed_mutex> lock_ano(ontology_->anonymous_graph_.mutex_);

    for(auto* rule_branch : ontology_->rule_graph_.get())
    {
      std::vector<RuleResult_t> results_resolve;
      std::vector<index_t> empty_accu(rule_branch->to_variables_.size(), index_t()); // need to initialize each index at 0
      results_resolve = resolveBody(rule_branch, rule_branch->rule_body_, empty_accu);

      // std::cout << "For rule " << rule_branch->value() << " results are: " << std::endl;

      // for(auto& rs : results_resolve)
      // {
      //   std::cout << "--> ";
      //   for(size_t i = 0; i < rs.assigned_result.size(); i++)
      //     std::cout << "[" << i << "]" << rs.assigned_result[i] << " ";
      //   std::cout << std::endl;
      // }

      for(auto& solution : results_resolve) // resolve the consequent for each found solution
        resolveHead(rule_branch->rule_head_, solution, rule_branch);
    }
  }

  void ReasonerRule::resolveHead(const std::vector<RuleTriplet_t>& atoms, RuleResult_t& solution, RuleBranch* rule)
  {
    for(const auto& atom : atoms)
    {
      switch(atom.atom_type_)
      {
      case class_atom:
        addInferredClassAtom(atom, solution, rule);
        break;
      case object_atom:
        addInferredObjectAtom(atom, solution, rule);
        break;
      case data_atom:
        addInferredDataAtom(atom, solution, rule);
        break;
      case builtin_atom:
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

  void ReasonerRule::addInferredClassAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule) // maybe not const since we mark the updates
  {
    IndividualBranch* involved_indiv = nullptr;

    if(triplet.subject.is_variable && (solution.assigned_result[triplet.subject.variable_id] != 0))
      involved_indiv = ontology_->individual_graph_.findBranch(solution.assigned_result[triplet.subject.variable_id]);
    else
      involved_indiv = triplet.subject.indiv_value;

    const bool is_already_a = std::any_of(involved_indiv->is_a_.cbegin(), involved_indiv->is_a_.cend(), [triplet](const auto& is_a) { return is_a.elem == triplet.class_predicate; });

    if(involved_indiv != nullptr)
    {
      // std::cout << "adding inferred " << triplet.class_predicate->value() << "(" << involved_indiv->value() << ")" << std::endl;
      if((is_already_a == false) && (checkClassesDisjointess(involved_indiv, triplet.class_predicate) == false))
      {
        involved_indiv->is_a_.emplaceBack(triplet.class_predicate, 1.0, true); // adding the emplaceBack so that the is_a get in updated mode
        triplet.class_predicate->individual_childs_.emplace_back(involved_indiv, 1.0, true);

        involved_indiv->nb_updates_++;
        triplet.class_predicate->nb_updates_++;

        // insert all explanations since they all are the reason why it has been inferred
        involved_indiv->is_a_.back().explanation.insert(involved_indiv->is_a_.back().explanation.end(),
                                                        solution.explanations.begin(),
                                                        solution.explanations.end());
        // add the pointer to the rule_branch used to make that inference
        involved_indiv->is_a_.back().used_rule = rule;

        for(auto& used : solution.triplets_used)
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

  void ReasonerRule::addInferredObjectAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule)
  {
    IndividualBranch* involved_indiv_from = nullptr;
    IndividualBranch* involved_indiv_on = nullptr;

    if(triplet.subject.is_variable && (solution.assigned_result[triplet.subject.variable_id] != 0))
      involved_indiv_from = ontology_->individual_graph_.findBranch(solution.assigned_result[triplet.subject.variable_id]);
    else
      involved_indiv_from = triplet.subject.indiv_value;
    if(triplet.object.is_variable && (solution.assigned_result[triplet.object.variable_id] != 0))
      involved_indiv_on = ontology_->individual_graph_.findBranch(solution.assigned_result[triplet.object.variable_id]);
    else
      involved_indiv_on = triplet.object.indiv_value;

    if(involved_indiv_from != nullptr && involved_indiv_on != nullptr)
    {
      // std::cout << "adding inferred :" << triplet.object_predicate->value() << "(" << involved_indiv_from->value() << ", " << involved_indiv_on->value() << ")" << std::endl;
      if(ontology_->individual_graph_.relationExists(involved_indiv_from, triplet.object_predicate, involved_indiv_on) == false)
      {
        int relation_index = ontology_->individual_graph_.addRelation(involved_indiv_from, triplet.object_predicate, involved_indiv_on, 1.0, true, false);
        involved_indiv_from->nb_updates_++;

        involved_indiv_from->object_relations_[relation_index].explanation.insert(involved_indiv_from->object_relations_[relation_index].explanation.end(),
                                                                                  solution.explanations.begin(),
                                                                                  solution.explanations.end());
        involved_indiv_from->object_relations_[relation_index].used_rule = rule;

        for(auto& used : solution.triplets_used)
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

  void ReasonerRule::addInferredDataAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule)
  {
    IndividualBranch* involved_indiv_from = nullptr;
    LiteralNode* involved_literal_on = nullptr;

    if(triplet.subject.is_variable && (solution.assigned_result[triplet.subject.variable_id] != 0))
      involved_indiv_from = ontology_->individual_graph_.findBranch(solution.assigned_result[triplet.subject.variable_id]);
    else
      involved_indiv_from = triplet.subject.indiv_value;
    if(triplet.object.is_variable && (solution.assigned_result[triplet.object.variable_id] != 0))
      involved_literal_on = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-solution.assigned_result[triplet.object.variable_id]));
    else
      involved_literal_on = triplet.object.datatype_value;

    if(involved_indiv_from != nullptr && involved_literal_on != nullptr)
    {
      // std::cout << "adding inference : " << triplet.data_predicate->value() << "(" << involved_indiv_from->value() << ", " << involved_literal_on->value() << ")" << std::endl;
      if(ontology_->individual_graph_.relationExists(involved_indiv_from, triplet.data_predicate, involved_literal_on) == false)
      {
        int relation_index = ontology_->individual_graph_.addRelation(involved_indiv_from, triplet.data_predicate, involved_literal_on, 1.0, true, false);
        involved_indiv_from->nb_updates_++;

        involved_indiv_from->data_relations_[relation_index].explanation.insert(involved_indiv_from->data_relations_[relation_index].explanation.end(),
                                                                                solution.explanations.begin(),
                                                                                solution.explanations.end());
        involved_indiv_from->data_relations_[relation_index].used_rule = rule;

        for(auto& used : solution.triplets_used)
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
        if(accu[var_index] != 0) // the variable has already been assigned with a value
        {
          if((accu[var_index] > 0) && (accu[var_index] != value.indiv->get())) // if the previously assigned indiv value and the new one are different, we continue
            continue;
          else if((accu[var_index] < 0) && (accu[var_index] != value.literal->get())) // if the previously assigned literal value and the new one are different, we continue
            continue;
        }
        else
        {
          if(value.indiv != nullptr)
            new_accu[var_index] = value.indiv->get();
          else if(value.literal != nullptr)
            new_accu[var_index] = value.literal->get();
          else
            std::cout << "No value was returned" << std::endl;
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
    case class_atom:
      resolveClassAtom(triplet, accu, var_index, values);
      break;
    case object_atom:
      resolveObjectAtom(triplet, accu, var_index, values);
      break;
    case data_atom:
      resolveDataAtom(triplet, accu, var_index, values);
      break;
    case builtin_atom:
      resolveBuiltinAtom(triplet, accu, var_index, values);
      break;
    default:
      break;
    }
  }

  void ReasonerRule::resolveClassAtom(const RuleTriplet_t& triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    // std::cout << "Resolving Class atom : " << triplet.toString() << std::endl;
    var_index = triplet.subject.variable_id;
    if(triplet.subject.is_variable == true)
    {
      if(accu[var_index] == 0) // has no previous value
        getType(triplet.class_predicate, values);
      else
      {
        IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(accu[var_index]);
        auto res = isA(involved_indiv, triplet.class_predicate);
        if(res.empty() == false)
          values.emplace_back(std::move(res));
      }
    }
    else
    {
      // atom is not variable, so we check the individual in the triplet
      auto res = isA(triplet.subject.indiv_value, triplet.class_predicate);
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

  void ReasonerRule::constructResult(const std::string& concept,
                                     const RelationsWithInductions<IndividualElement>& relation,
                                     size_t index, IndivResult_t& res)
  {
    std::string explanation = concept + "|sameAs|" + relation.at(index).elem->value();
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
    // std::cout << "Resolving Object atom : " << triplet.toString() << std::endl;
    if(!triplet.subject.is_variable && !triplet.object.is_variable)
    {
      values = getOnObject(triplet, triplet.object.indiv_value->get());
    }
    else if(triplet.subject.is_variable && !triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      if(accu[var_index] == 0)
        values = getFromObject(triplet);
      else
      {
        triplet.subject.indiv_value = ontology_->individual_graph_.findBranch(accu[var_index]);
        var_index = triplet.object.variable_id;
        values = getOnObject(triplet, triplet.object.indiv_value->get());
      }
    }
    else if(!triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.object.variable_id;
      values = getOnObject(triplet, accu[var_index]);
    }
    else if(triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      if(accu[var_index] != 0)
      {
        triplet.subject.indiv_value = ontology_->individual_graph_.findBranch(accu[var_index]);
        var_index = triplet.object.variable_id;
        values = getOnObject(triplet, accu[var_index]);
      }
      else if(accu[triplet.object.variable_id] != 0)
      {
        triplet.object.indiv_value = ontology_->individual_graph_.findBranch(accu[triplet.object.variable_id]);
        values = getFromObject(triplet);
      }
      else
        std::cout << "no variable bounded to any of the variable fields" << std::endl;
    }
  }

  std::vector<IndivResult_t> ReasonerRule::getFromObject(const RuleTriplet_t& triplet)
  {
    std::vector<IndivResult_t> res;

    if(triplet.object.indiv_value->same_as_.empty())
    {
      for(auto& indiv : ontology_->individual_graph_.all_branchs_)
        for(size_t i = 0; i < indiv->object_relations_.size(); i++)
        {
          const auto& relations = indiv->object_relations_;
          if(relations.at(i).second == triplet.object.indiv_value)
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
      for(auto& indiv : ontology_->individual_graph_.all_branchs_)
        for(size_t i = 0; i < indiv->object_relations_.size(); i++)
        {
          for(size_t j = 0; j < triplet.object.indiv_value->same_as_.size(); j++)
          {
            const auto& relations = indiv->object_relations_;
            if(relations.at(i).second == triplet.object.indiv_value->same_as_[j].elem)
            {
              IndivResult_t local_res;
              if((relations.at(i).first == triplet.object_predicate) ||
                 isA(relations.at(i).first->value(), triplet.object_predicate, relations.at(i).first->mothers_, local_res))
              {
                local_res.indiv = indiv;
                constructResult(triplet.object.indiv_value->same_as_[i].elem->value(), relations, i, local_res);

                if(triplet.object.indiv_value != triplet.object.indiv_value->same_as_[i].elem)
                  constructResult(triplet.object.indiv_value->value(), triplet.object.indiv_value->same_as_, j, local_res);

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
    IndividualBranch* indiv = triplet.subject.indiv_value;
    IndividualBranch* selector_ptr = nullptr;
    if(selector != 0)
      selector_ptr = ontology_->individual_graph_.findBranch(selector);

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
        IndividualBranch* same = indiv->same_as_[s].elem;
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
    if(!triplet.subject.is_variable && !triplet.object.is_variable)
    {
      values = getOnData(triplet, triplet.object.datatype_value->get());
    }
    else if(triplet.subject.is_variable && !triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      if(accu[var_index] == 0)
        values = getFromData(triplet);
      else
      {
        triplet.subject.indiv_value = ontology_->individual_graph_.findBranch(accu[var_index]);
        var_index = triplet.object.variable_id;
        values = getOnData(triplet, triplet.object.datatype_value->get());
      }
    }
    else if(!triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.object.variable_id;
      values = getOnData(triplet, accu[var_index]);
    }
    else if(triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      if(accu[var_index] != 0)
      {
        triplet.subject.indiv_value = ontology_->individual_graph_.findBranch(accu[var_index]);
        var_index = triplet.object.variable_id;
        values = getOnData(triplet, accu[var_index]);
      }
      else if(accu[triplet.object.variable_id] != 0)
      {
        triplet.object.datatype_value = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-accu[triplet.object.variable_id]));
        values = getFromData(triplet);
      }
      else
        std::cout << "no variable bounded to any of the variable fields" << std::endl;
    }
  }

  std::vector<IndivResult_t> ReasonerRule::getFromData(const RuleTriplet_t& triplet)
  {
    std::vector<IndivResult_t> res;

    for(auto& indiv : ontology_->individual_graph_.all_branchs_)
      for(size_t i = 0; i < indiv->data_relations_.size(); i++)
      {
        const auto& relations = indiv->data_relations_;
        if(relations.at(i).second == triplet.object.datatype_value)
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
    IndividualBranch* indiv = triplet.subject.indiv_value;

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
        IndividualBranch* same = indiv->same_as_[s].elem;
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

    var_index = triplet.subject.variable_id;

    if(accu[triplet.subject.variable_id] != 0)
      subject_ptr = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-accu[triplet.subject.variable_id]));
    else if(triplet.subject.datatype_value != nullptr)
      subject_ptr = triplet.subject.datatype_value;
    else
    {
      std::cout << "No value for arg1 of the builtin :" << triplet.builtin.builtin_str_ << std::endl;
      return;
    }

    if(accu[triplet.object.variable_id] != 0)
      object_ptr = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-accu[triplet.object.variable_id]));
    else if(triplet.object.datatype_value != nullptr)
      object_ptr = triplet.object.datatype_value;
    else
    {
      std::cout << "No value for arg2 of the builtin :" << triplet.builtin.builtin_str_ << std::endl;
      return;
    }

    if((subject_ptr != nullptr) && (object_ptr != nullptr))
    {
      if((subject_ptr->type_ == "string") && (object_ptr->type_ == "string") &&
         (resolveStringBuiltinAtom(triplet.builtin.builtin_type_, subject_ptr, object_ptr) == true))
      {
        res.literal = subject_ptr;
        std::string explanation = triplet.builtin.builtin_str_ + "(" + subject_ptr->value() + "," + object_ptr->value();
        res.explanations.emplace_back(explanation);
        values.emplace_back(std::move(res));
      }
      else if(resolveNumericalBuiltinAtom(triplet.builtin.builtin_type_, subject_ptr, object_ptr) == true)
      {
        res.literal = subject_ptr;
        std::string explanation = triplet.builtin.builtin_str_ + "(" + subject_ptr->value() + "," + object_ptr->value();
        res.explanations.emplace_back(explanation);
        values.emplace_back(std::move(res));
      }
    }
  }

  bool ReasonerRule::resolveNumericalBuiltinAtom(BuiltinType_e builtin_type, LiteralNode* subject, LiteralNode* object)
  {
    try
    {
      IndivResult_t res;

      const double subject_cast = stod(subject->value_);
      const double object_cast = stod(object->value_);

      switch(builtin_type)
      {
      case greater_than:
        return subject_cast > object_cast;
      case greater_than_or_equal:
        return subject_cast >= object_cast;
      case less_than:
        return subject_cast < object_cast;
      case less_than_or_equal:
        return subject_cast <= object_cast;
      case equal:
        return subject_cast == object_cast;
      case not_equal:
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

  bool ReasonerRule::resolveStringBuiltinAtom(BuiltinType_e builtin_type, LiteralNode* subject, LiteralNode* object)
  {
    switch(builtin_type)
    {
    case equal:
      return subject->value() == object->value();
    case not_equal:
      return subject->value() != object->value();
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