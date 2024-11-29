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

      std::cout << "For rule " << rule_branch->value() << "results are:" << std::endl;
      for(auto& rs : results_resolve)
      {
        std::cout << "--> ";
        for(size_t i = 0; i < rs.assigned_result.size(); i++)
          std::cout << "[" << i << "]" << rs.assigned_result[i] << " ";
        std::cout << std::endl;
      }

      for(auto& solution : results_resolve) // resolve the consequent for each found solution
        resolveHead(rule_branch->rule_head_, solution, rule_branch);
    }
  }

  void ReasonerRule::resolveHead(const std::vector<RuleTriplet_t>& atoms, RuleResult_t& solution, RuleBranch* rule)
  {
    for(auto& atom : atoms)
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

  bool ReasonerRule::relationExists(IndividualBranch* indiv_from, ObjectPropertyBranch* property, IndividualBranch* indiv_on)
  {
    for(auto& relation : indiv_from->object_relations_)
    {
      if(relation.second->get() == indiv_on->get())
      {
        std::unordered_set<ObjectPropertyBranch*> down_properties;
        ontology_->object_property_graph_.getDownPtr(property, down_properties);
        if(down_properties.find(relation.first) != down_properties.end())
          return true;
      }
    }
    return false;
  }

  bool ReasonerRule::relationExists(IndividualBranch* indiv_from, DataPropertyBranch* property, LiteralNode* literal_on)
  {
    for(auto& relation : indiv_from->data_relations_)
    {
      if(relation.second->get() == literal_on->get())
      {
        std::unordered_set<DataPropertyBranch*> down_properties;
        ontology_->data_property_graph_.getDownPtr(property, down_properties);
        if(down_properties.find(relation.first) != down_properties.end())
          return true;
      }
    }
    return false;
  }

  void ReasonerRule::addInferredClassAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule) // maybe not const since we mark the updates
  {
    // the value we need to find is solution.assigned_result[triplet.subject.variable_id]

    IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(solution.assigned_result[triplet.subject.variable_id]);
    const bool is_already_a = std::any_of(involved_indiv->is_a_.cbegin(), involved_indiv->is_a_.cend(), [triplet](const auto& is_a) { return is_a.elem == triplet.class_predicate; });

    if(is_already_a == false && (checkClassesDisjointess(involved_indiv, triplet.class_predicate) == false))
    {
      involved_indiv->is_a_.emplaceBack(triplet.class_predicate, 1.0, true); // adding the emplaceBack so that the is_a get in updated mode
      triplet.class_predicate->individual_childs_.emplace_back(IndividualElement(involved_indiv, 1.0, true));

      involved_indiv->nb_updates_++;
      triplet.class_predicate->nb_updates_++;

        // insert all explanations since they all are the reason why it has been inferred
        involved_indiv->is_a_.back().explanation.insert(involved_indiv->is_a_.back().explanation.end(), solution.explanations.begin(), solution.explanations.end());
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
      std::cout << involved_indiv->is_a_.back().getExplanation() << std::endl;
      nb_update++;

      explanations_.emplace_back("[ADD]" + involved_indiv->value() + "|isA|" + triplet.class_predicate->value(),
                                 "[ADD]" + involved_indiv->is_a_.back().getExplanation());
    }
  }

  void ReasonerRule::addInferredObjectAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule)
  {
    IndividualBranch* involved_indiv_from = ontology_->individual_graph_.findBranch(solution.assigned_result[triplet.subject.variable_id]);
    IndividualBranch* involved_indiv_on = ontology_->individual_graph_.findBranch(solution.assigned_result[triplet.object.variable_id]);

    if(!relationExists(involved_indiv_from, triplet.object_predicate, involved_indiv_on))
    {
      ontology_->individual_graph_.addRelation(involved_indiv_from, triplet.object_predicate, involved_indiv_on, 1.0, true, false);
      involved_indiv_from->nb_updates_++;

        involved_indiv_from->object_relations_.back().explanation.insert(involved_indiv_from->object_relations_.back().explanation.end(), solution.explanations.begin(), solution.explanations.end());
        involved_indiv_from->object_relations_.back().used_rule = rule;

      for(auto& used : solution.triplets_used)
      {
        auto* object_triplet = used.getObject();
        // auto* inheritace_triplet = used.getInheritance();
        if(object_triplet->exist(involved_indiv_from, triplet.object_predicate, involved_indiv_on) == false)
        {
          object_triplet->push(involved_indiv_from, triplet.object_predicate, involved_indiv_on);
          involved_indiv_from->object_relations_.relations.back().induced_traces.emplace_back(object_triplet); // need to add every X_triplet used to induced_traces
          // involved_indiv_from->object_relations_.relations.back().induced_traces.emplace_back(inheritace_triplet);
        }
      }

      nb_update++;
      explanations_.emplace_back("[ADD]" + involved_indiv_from->value() + "|" + triplet.object_predicate->value() + "|" + involved_indiv_on->value(),
                                 "[ADD]" + involved_indiv_from->object_relations_.back().getExplanation());
    }
  }

  void ReasonerRule::addInferredDataAtom(const RuleTriplet_t& triplet, RuleResult_t& solution, RuleBranch* rule)
  {
    // std::cout << " data : index of variable subject : " << solution.assigned_result[triplet.subject.variable_id] << std::endl;
    IndividualBranch* involved_indiv_from = ontology_->individual_graph_.findBranch(solution.assigned_result[triplet.subject.variable_id]);
    // std::cout << " data : index of variable object : " << solution.assigned_result[triplet.object.variable_id] << std::endl;
    if(solution.assigned_result[triplet.object.variable_id] != 0) // because of builtin usupported atom
    {
      LiteralNode* involved_literal_on = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-solution.assigned_result[triplet.object.variable_id]));

      if(!relationExists(involved_indiv_from, triplet.data_predicate, involved_literal_on))
      {
        ontology_->individual_graph_.addRelation(involved_indiv_from, triplet.data_predicate, involved_literal_on, 1.0, true);
        involved_indiv_from->nb_updates_++;

        involved_indiv_from->data_relations_.back().explanation.insert(involved_indiv_from->is_a_.back().explanation.end(), solution.explanations.begin(), solution.explanations.end());
        involved_indiv_from->data_relations_.back().used_rule = rule;

        for(auto& used : solution.triplets_used)
        {
          auto* data_triplet = used.getData();
          if(data_triplet->exist(involved_indiv_from, triplet.data_predicate, involved_literal_on) == false)
          {
            data_triplet->push(involved_indiv_from, triplet.data_predicate, involved_literal_on); // write into has_induced_data_relation for the newly asserted relation
            involved_indiv_from->data_relations_.relations.back().induced_traces.emplace_back(data_triplet);
          }
        }

        nb_update++;
        explanations_.emplace_back("[ADD]" + involved_indiv_from->value() + "|" + triplet.data_predicate->value() + "|" + involved_literal_on->value(),
                                   "[ADD]" + involved_indiv_from->data_relations_.back().getExplanation());
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

  void ReasonerRule::resolveAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
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
      break;

    default:
      break;
    }
  }

  void ReasonerRule::resolveClassAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    var_index = triplet.subject.variable_id;
    if(triplet.subject.is_variable == true)
    {
      if(accu[var_index] == 0) // has no previous value
        values = getType(triplet.class_predicate);
      else if(ontology_->individual_graph_.isA(accu[var_index], triplet.class_predicate->get()) == true)
      {
        // has a previous value, so we evaluate it
        IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(accu[var_index]);
        values.emplace_back(checkInstantiatedTriplet(involved_indiv, triplet.class_predicate));
      }
    }
    else if(ontology_->individual_graph_.isA(triplet.subject.indiv_value, triplet.class_predicate->value()) == true)
    {
      // atom is not variable, so we check the individual in the triplet
      values.emplace_back(checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.class_predicate));
    }
  }

  void ReasonerRule::resolveObjectAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    if(!triplet.subject.is_variable && !triplet.object.is_variable)
    {
      IndivResult_t used = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.object_predicate, triplet.object.indiv_value, true); // true because the indiv matching the atom is subject
      if(used.empty() == false)
        values.push_back(used);
    }
    else if(triplet.subject.is_variable && !triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      values = getFrom(triplet, accu[var_index]);
    }
    else if(!triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.object.variable_id;
      values = getOn(triplet, accu[var_index]);
    }
    else if(triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      if(accu[var_index] != 0)
      {
        triplet.subject.indiv_value = ontology_->individual_graph_.findBranch(accu[var_index]);
        var_index = triplet.object.variable_id;
        values = getOn(triplet, accu[var_index]);
      }
      else if(accu[triplet.object.variable_id] != 0)
      {
        triplet.object.indiv_value = ontology_->individual_graph_.findBranch(accu[triplet.object.variable_id]);
        values = getFrom(triplet, 0);
      }
      else
        std::cout << "no variable bounded to any of the variable fields" << std::endl;
    }
  }

  void ReasonerRule::resolveDataAtom(RuleTriplet_t triplet, std::vector<index_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    if(!triplet.subject.is_variable && !triplet.object.is_variable)
    {
      IndivResult_t used = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, triplet.object.datatype_value, true); // true because the indiv matching the atom is subject
      if(used.empty() == false)
        values.push_back(used);
    }
    else if(triplet.subject.is_variable && !triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      values = getFrom(triplet, accu[var_index]);
    }
    else if(!triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.object.variable_id;
      values = getOn(triplet, accu[var_index]);
    }
    else if(triplet.subject.is_variable && triplet.object.is_variable)
    {
      var_index = triplet.subject.variable_id;
      if(accu[var_index] != 0)
      {
        triplet.subject.indiv_value = ontology_->individual_graph_.findBranch(accu[var_index]);
        var_index = triplet.object.variable_id;
        values = getOn(triplet, accu[var_index]);
      }
      else
      {
        if(accu[triplet.object.variable_id] != 0)
        {
          triplet.object.datatype_value = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-accu[triplet.object.variable_id]));
          values = getFrom(triplet, 0);
        }
        else
          std::cout << "no variable bounded to any of the variable fields" << std::endl;
      }
    }
  }

  std::vector<IndivResult_t> ReasonerRule::getFrom(RuleTriplet_t& triplet, const index_t& index_indiv_from) // works for literal and indiv
  {
    std::vector<IndivResult_t> res_from;
    std::unordered_set<IndividualBranch*> candidates_indivs_from;
    IndivResult_t used_solution;

    if(index_indiv_from == 0)
    {
      if(triplet.object_predicate != nullptr)
        candidates_indivs_from = getFrom(triplet.object_predicate, triplet.object.indiv_value->get());
      else
        candidates_indivs_from = getFrom(triplet.data_predicate, triplet.object.datatype_value->get());

      res_from.reserve(candidates_indivs_from.size());
      for(auto* indiv_from : candidates_indivs_from) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
      {
        if(triplet.object_predicate != nullptr)
          used_solution = checkInstantiatedTriplet(indiv_from, triplet.object_predicate, triplet.object.indiv_value, true);
        else
          used_solution = checkInstantiatedTriplet(indiv_from, triplet.data_predicate, triplet.object.datatype_value, true);
        if(used_solution.empty() == false)
          res_from.push_back(used_solution);
      }
    }
    else
    { // here we revert the problem since we know what we are looking for
      if(triplet.object_predicate != nullptr)
      {
        std::unordered_set<IndividualBranch*> candidates_indivs_on = getOn(index_indiv_from, triplet.object_predicate);
        if(std::find(candidates_indivs_on.begin(), candidates_indivs_on.end(), triplet.object.indiv_value) != candidates_indivs_on.end())
        {
          IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_indiv_from);
          used_solution = checkInstantiatedTriplet(involved_indiv, triplet.object_predicate, triplet.object.indiv_value, true);
          if(used_solution.empty() == false)
            res_from.push_back(used_solution);
        }
      }
      else
      {
        std::unordered_set<LiteralNode*> candidates_literals_on = getOn(index_indiv_from, triplet.data_predicate);
        if(std::find(candidates_literals_on.begin(), candidates_literals_on.end(), triplet.object.datatype_value) != candidates_literals_on.end())
        {
          IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_indiv_from);
          used_solution = checkInstantiatedTriplet(involved_indiv, triplet.data_predicate, triplet.object.datatype_value, true);
          if(used_solution.empty() == false)
            res_from.push_back(used_solution);
        }
      }
    }
    return res_from;
  }
  // returns IndivResults with IndividualBranch, explanation and triplets used
  // std::vector<IndivResult_t> ReasonerRule::getFromObject(RuleTriplet_t& triplet, const index_t& index_indiv_from)
  // {
  //   std::vector<IndivResult_t> res_from;

  //   if(index_indiv_from == 0)
  //   {
  //     std::unordered_set<IndividualBranch*> candidates_indivs_from = getFrom(triplet.object_predicate, triplet.object.indiv_value->get());
  //     res_from.reserve(candidates_indivs_from.size());
  //     for(auto* indiv_from : candidates_indivs_from) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
  //     {
  //       IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from, triplet.object_predicate, triplet.object.indiv_value, true);
  //       if(used_solution.empty() == false)
  //         res_from.push_back(used_solution);
  //     }
  //   }
  //   else
  //   {
  //     // here we revert the problem since we know what we are looking for
  //     std::unordered_set<IndividualBranch*> candidates_indivs_on = getOn(index_indiv_from, triplet.object_predicate);
  //     if(std::find(candidates_indivs_on.begin(), candidates_indivs_on.end(), triplet.object.indiv_value) != candidates_indivs_on.end())
  //     {
  //       IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_indiv_from);
  //       IndivResult_t used_solution = checkInstantiatedTriplet(involved_indiv, triplet.object_predicate, triplet.object.indiv_value, true);
  //       if(used_solution.empty() == false)
  //         res_from.push_back(used_solution);
  //     }
  //   }
  //   return res_from;
  // }

  // std::vector<IndivResult_t> ReasonerRule::getFromData(RuleTriplet_t& triplet, const index_t& index_indiv_from)
  // {
  //   std::vector<IndivResult_t> res_from;

  //   if(index_indiv_from == 0)
  //   {
  //     std::unordered_set<IndividualBranch*> candidates_indivs_from = getFrom(triplet.data_predicate, triplet.object.datatype_value->get());
  //     res_from.reserve(candidates_indivs_from.size());
  //     for(auto* indiv_from : candidates_indivs_from) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
  //     {
  //       IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from, triplet.data_predicate, triplet.object.datatype_value, true);
  //       if(used_solution.empty() == false)
  //         res_from.push_back(used_solution);
  //     }
  //   }
  //   else
  //   {
  //     // here we revert the problem since we know what we are looking for
  //     std::unordered_set<LiteralNode*> candidates_literals_on = getOn(index_indiv_from, triplet.data_predicate);
  //     if(std::find(candidates_literals_on.begin(), candidates_literals_on.end(), triplet.object.datatype_value) != candidates_literals_on.end())
  //     {
  //       IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_indiv_from);
  //       IndivResult_t used_solution = checkInstantiatedTriplet(involved_indiv, triplet.data_predicate, triplet.object.datatype_value, true);
  //       if(used_solution.empty() == false)
  //         res_from.push_back(used_solution);
  //     }
  //   }
  //   return res_from;
  // }

  std::vector<IndivResult_t> ReasonerRule::getOn(RuleTriplet_t& triplet, const index_t& index_resource_on)
  {
    std::vector<IndivResult_t> res_on;
    IndivResult_t used_solution;

    if(index_resource_on != 0)
    {
      if(triplet.object_predicate != nullptr)
      {
        IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_resource_on);
        used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.object_predicate, involved_indiv, false);
      }
      else
      {
        LiteralNode* involved_literal = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-index_resource_on));
        used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, involved_literal, false);
      }
      if(used_solution.empty() == false)
        res_on.push_back(used_solution);
      else
        return {};
    }
    else
    {
      if(triplet.object_predicate != nullptr)
      {
        std::unordered_set<IndividualBranch*> candidates_indivs_on = getOn(triplet.subject.indiv_value->get(), triplet.object_predicate);
        if(candidates_indivs_on.empty() == false)
        {
          res_on.reserve(candidates_indivs_on.size());
          for(auto* indiv_on : candidates_indivs_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
          {
            IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.object_predicate, indiv_on, false);
            if(used_solution.empty() == false)
              res_on.push_back(used_solution);
          }
        }
      }
      else
      {
        std::unordered_set<LiteralNode*> candidates_literals_on = getOn(triplet.subject.indiv_value->get(), triplet.data_predicate);
        if(candidates_literals_on.empty() == false)
        {
          res_on.reserve(candidates_literals_on.size());
          for(auto* literal_on : candidates_literals_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
          {
            IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, literal_on, false);
            if(used_solution.empty() == false)
              res_on.push_back(used_solution);
          }
        }
      }
    }
    return res_on;
  }

  // returns IndivResults with either IndividualBranch or LiteralNode, explanation and triplets used
  // std::vector<IndivResult_t> ReasonerRule::getOnObject(RuleTriplet_t& triplet, const index_t& index_indiv_on)
  // {
  //   std::vector<IndivResult_t> res_on;

  //   if(index_indiv_on != 0)
  //   {
  //     // check if triplet.subject.indiv_value, predicate, indiv_on.indiv holds
  //     IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_indiv_on);
  //     IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.object_predicate, involved_indiv, false);
  //     if(used_solution.empty() == false)
  //       res_on.push_back(used_solution);
  //     else
  //       return {};
  //   }
  //   else
  //   {
  //     // return all individuals matching the (triplet.subject.indiv_value, property, X)
  //     std::unordered_set<IndividualBranch*> candidates_indivs_on = getOn(triplet.subject.indiv_value->get(), triplet.object_predicate);
  //     if(candidates_indivs_on.empty() == false)
  //     {
  //       res_on.reserve(candidates_indivs_on.size());
  //       for(auto* indiv_on : candidates_indivs_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
  //       {
  //         IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.object_predicate, indiv_on, false);
  //         if(used_solution.empty() == false)
  //           res_on.push_back(used_solution);
  //       }
  //     }
  //   }
  //   return res_on;
  // }

  // std::vector<IndivResult_t> ReasonerRule::getOnData(RuleTriplet_t& triplet, const index_t& index_literal_on)
  // {
  //   std::vector<IndivResult_t> res;

  //   if(index_literal_on != 0)
  //   {
  //     LiteralNode* involved_literal = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-index_literal_on));
  //     IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, involved_literal, false);
  //     if(used_solution.empty() == false)
  //       res.push_back(used_solution);
  //     else
  //       return {};
  //   }
  //   else
  //   {
  //     // return all literals matching the (triplet.subject.indiv_value, property, X)
  //     std::unordered_set<LiteralNode*> candidates_literals_on = getOn(triplet.subject.indiv_value->get(), triplet.data_predicate);
  //     if(candidates_literals_on.empty() == false)
  //     {
  //       res.reserve(candidates_literals_on.size());
  //       for(auto* literal_on : candidates_literals_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
  //       {
  //         IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, literal_on, false);
  //         if(used_solution.empty() == false)
  //           res.push_back(used_solution);
  //       }
  //     }
  //   }
  //   return res;
  // }

  // returns IndividualBranch without explanations
  std::unordered_set<IndividualBranch*> ReasonerRule::getFrom(ObjectPropertyBranch* property, const index_t& index_indiv_on)
  {
    std::unordered_set<IndividualBranch*> indivs_from;
    const std::unordered_set<index_t> object_properties = ontology_->object_property_graph_.getDownId(property->get());

    for(auto& indiv_i : ontology_->individual_graph_.all_branchs_)
      for(const IndivObjectRelationElement& relation : indiv_i->object_relations_)
        for(const index_t id : object_properties)
          if(relation.first->get() == id)
          {
            if(relation.second->get() == index_indiv_on)
            {
              indivs_from.insert(indiv_i);
              break;
            }
          }

    return indivs_from;
  }

  std::unordered_set<IndividualBranch*> ReasonerRule::getFrom(DataPropertyBranch* property, const index_t& index_literal_on)
  {
    std::unordered_set<IndividualBranch*> indivs_from;
    const std::unordered_set<index_t> data_properties = ontology_->data_property_graph_.getDownId(property->get());

    for(auto& indiv_i : ontology_->individual_graph_.all_branchs_)
      for(const IndivDataRelationElement& relation : indiv_i->data_relations_)
        for(const index_t id : data_properties)
          if(relation.first->get() == id)
          {
            if(relation.second->get() == index_literal_on)
            {
              indivs_from.insert(indiv_i);
              break;
            }
          }

    return indivs_from;
  }

  // returns either IndividualBranch or Literalode without explanations
  std::unordered_set<IndividualBranch*> ReasonerRule::getOn(const index_t& index_indiv_from, ObjectPropertyBranch* predicate)
  {
    IndividualBranch* indiv_from = ontology_->individual_graph_.findBranch(index_indiv_from);
    std::unordered_set<IndividualBranch*> indivs_on;
    std::unordered_set<index_t> object_properties = ontology_->object_property_graph_.getDownId(predicate->get());

    if(object_properties.empty() == false)
    {
      if(indiv_from->same_as_.empty() == false)
      {
        for(auto& same_indiv : indiv_from->same_as_)
        {
          for(const IndivObjectRelationElement& relation : same_indiv.elem->object_relations_)
            for(const index_t id : object_properties)
              if(relation.first->get() == id)
              {
                indivs_on.insert(relation.second);
                break;
              }
        }
      }
      else
      {
        for(const IndivObjectRelationElement& relation : indiv_from->object_relations_)
          for(const index_t id : object_properties)
            if(relation.first->get() == id)
            {
              indivs_on.insert(relation.second);
              break;
            }
      }
    }

    return indivs_on;
  }

  std::unordered_set<LiteralNode*> ReasonerRule::getOn(const index_t& index_indiv_from, DataPropertyBranch* predicate)
  {
    IndividualBranch* indiv_from = ontology_->individual_graph_.findBranch(index_indiv_from);
    std::unordered_set<LiteralNode*> literals_on;
    std::unordered_set<index_t> data_properties = ontology_->data_property_graph_.getDownId(predicate->get());

    if(data_properties.empty() == false)
    {
      if(indiv_from->same_as_.empty() == false)
      {
        for(auto& same_indiv : indiv_from->same_as_)
        {
          for(const IndivDataRelationElement& relation : same_indiv.elem->data_relations_)
            for(const index_t id : data_properties)
              if(relation.first->get() == id)
              {
                literals_on.insert(relation.second);
                break;
              }
        }
      }
      else
      {
        for(const IndivDataRelationElement& relation : indiv_from->data_relations_)
          for(const index_t id : data_properties)
            if(relation.first->get() == id)
            {
              literals_on.insert(relation.second);
              break;
            }
      }
    }

    return literals_on;
  }

  std::vector<IndivResult_t> ReasonerRule::getType(ClassBranch* class_selector)
  {
    std::unordered_set<IndividualBranch*> indiv_res = ontology_->individual_graph_.getType(class_selector);
    std::vector<IndivResult_t> indiv_of_class;
    indiv_of_class.reserve(indiv_res.size());

    for(auto* indiv : indiv_res)
      indiv_of_class.emplace_back(checkInstantiatedTriplet(indiv, class_selector));

    return indiv_of_class;
  }

  // check the existence of the triplet and compute the explanation
  IndivResult_t ReasonerRule::checkInstantiatedTriplet(IndividualBranch* indiv, ClassBranch* class_selector)
  {
    IndivResult_t used_solution;

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
            if(existInInheritance(indiv->same_as_[i].elem->is_a_[j].elem, class_selector->get(), used_solution))
            {
              used_solution.indiv = indiv;

              if(class_selector->isHidden() == false)
              {
                std::string explanation = indiv->same_as_[i].elem->value() + "|isA|" + class_selector->value();
                used_solution.explanations.emplace_back(explanation);
              }
              else
              {
                const auto& hidden_explanation = indiv->same_as_[i].elem->is_a_[j].explanation;
                used_solution.explanations.insert(used_solution.explanations.end(),
                                                  hidden_explanation.cbegin(),
                                                  hidden_explanation.cend());
              }

              used_solution.used_triplets.emplace_back(indiv->same_as_[i].elem->is_a_.has_induced_inheritance_relations[j],
                                                       indiv->same_as_[i].elem->is_a_.has_induced_object_relations[j],
                                                       indiv->same_as_[i].elem->is_a_.has_induced_data_relations[j]);

              std::string explanation = indiv->value() + "|sameAs|" + indiv->same_as_[i].elem->value();
              used_solution.explanations.emplace_back(explanation);

              used_solution.used_triplets.emplace_back(indiv->same_as_.has_induced_inheritance_relations[i],
                                                       indiv->same_as_.has_induced_object_relations[i],
                                                       indiv->same_as_.has_induced_data_relations[i]);

              return used_solution;
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
        if(existInInheritance(indiv->is_a_[i].elem, class_selector->get(), used_solution))
        {
          if(class_selector->isHidden() == false)
          {
            std::string explanation = indiv->value() + "|isA|" + class_selector->value();
            used_solution.explanations.emplace_back(explanation);
          }
          else
          {
            const auto& hidden_explanation = indiv->is_a_[i].explanation;
            used_solution.explanations.insert(used_solution.explanations.end(),
                                              hidden_explanation.cbegin(),
                                              hidden_explanation.cend());
          }

          used_solution.indiv = indiv;
          used_solution.used_triplets.emplace_back(indiv->is_a_.has_induced_inheritance_relations[i],
                                                   indiv->is_a_.has_induced_object_relations[i],
                                                   indiv->is_a_.has_induced_data_relations[i]);

          return used_solution;
        }
      }
    }
    return IndivResult_t();
  }

  IndivResult_t ReasonerRule::checkInstantiatedTriplet(IndividualBranch* indiv_from, ObjectPropertyBranch* property_predicate, IndividualBranch* indiv_on, bool var_from)
  {
    IndivResult_t used_solution;

    int index = checkInstantiatedTriplet(indiv_from, property_predicate, indiv_on, indiv_from->object_relations_.relations, used_solution);

    if(index != -1)
    {
      if(var_from == true)
        used_solution.indiv = indiv_from; // mark the indiv used in this atom evaluation -> maybe useless
      else
        used_solution.indiv = indiv_on; // mark the indiv used in this atom evaluation

      used_solution.used_triplets.emplace_back(indiv_from->object_relations_.has_induced_inheritance_relations[index],
                                               indiv_from->object_relations_.has_induced_object_relations[index],
                                               indiv_from->object_relations_.has_induced_data_relations[index]);
    }

    return used_solution;
  }

  IndivResult_t ReasonerRule::checkInstantiatedTriplet(IndividualBranch* indiv_from, DataPropertyBranch* property_predicate, LiteralNode* literal_on, bool var_from) // if var_from = true, then used_solution used indiv_from, else it used resource_on
  {
    IndivResult_t used_solution;

    int index = checkInstantiatedTriplet(indiv_from, property_predicate, literal_on, indiv_from->data_relations_.relations, used_solution);

    if(index != -1)
    {
      if(var_from == true)
        used_solution.indiv = indiv_from; // mark the indiv used in this atom evaluation
      else
        used_solution.literal = literal_on; // mark the literal used in this atom evaluation

      used_solution.used_triplets.emplace_back(indiv_from->data_relations_.has_induced_inheritance_relations[index],
                                               indiv_from->data_relations_.has_induced_object_relations[index],
                                               indiv_from->data_relations_.has_induced_data_relations[index]);
    }

    return used_solution;
  }

  // check the match between either two individuals or literals
  bool ReasonerRule::checkValue(IndividualBranch* indiv_from, IndividualBranch* indiv_on, IndivResult_t& used)
  {
    if(indiv_from->same_as_.empty() == false)
    {
      const size_t same_size = indiv_from->same_as_.size();
      for(size_t j = 0; j < same_size; j++)
      {
        if(indiv_from->same_as_[j].elem->get() == indiv_on->get())
        {
          std::string explanation = indiv_from->value() + "|sameAs|" + indiv_on->value();
          used.explanations.emplace_back(explanation);

          used.used_triplets.emplace_back(indiv_from->same_as_.has_induced_inheritance_relations[j],
                                          indiv_from->same_as_.has_induced_object_relations[j],
                                          indiv_from->same_as_.has_induced_data_relations[j]);

          return true;
        }
      }
    }
    else if(indiv_from->get() == indiv_on->get())
      return true;

    return false;
  }

  bool ReasonerRule::checkValue(LiteralNode* literal_from, LiteralNode* literal_on, IndivResult_t& used)
  {
    (void)used;
    return (literal_from->get() == literal_on->get());
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