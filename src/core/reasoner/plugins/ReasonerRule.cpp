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
      results_resolve = resolve(rule_branch, rule_branch->rule_body_, empty_accu);

      // std::cout << "For rule " << rule_branch->value() << "results are:" << std::endl;
      // for(auto& rs : results_resolve)
      // {
      //   std::cout << "--> ";
      //   for(size_t i = 0; i < rs.assigned_result.size(); i++)
      //     std::cout << "[" << i << "]" << rs.assigned_result[i] << " ";
      //   std::cout << std::endl;
      // }
    }
  }

  std::vector<RuleResult_t> ReasonerRule::resolve(RuleBranch* rule_branch, std::vector<RuleTriplet_t>& atoms, std::vector<index_t>& accu)
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

        std::vector<RuleResult_t> local_res = resolve(rule_branch, new_atoms, new_accu); // new solutions updated with the new_accu

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
      values = getFromObject(triplet, accu[var_index]);
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
        values = getFromObject(triplet, 0);
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
      values = getFromData(triplet, accu[var_index]);
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
      else
      {
        if(accu[triplet.object.variable_id] != 0)
        {
          triplet.object.datatype_value = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-accu[triplet.object.variable_id]));
          values = getFromData(triplet, 0);
        }
        else
          std::cout << "no variable bounded to any of the variable fields" << std::endl;
      }
    }
  }

  // returns IndivResults with IndividualBranch, explanation and triplets used
  std::vector<IndivResult_t> ReasonerRule::getFromObject(RuleTriplet_t& triplet, const index_t& index_indiv_from)
  {
    std::vector<IndivResult_t> res_from;

    if(index_indiv_from == 0)
    {
      std::unordered_set<IndividualBranch*> candidates_indivs_from = getFrom(triplet.object_predicate, triplet.object.indiv_value->get());
      res_from.reserve(candidates_indivs_from.size());
      for(auto* indiv_from : candidates_indivs_from) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
      {
        IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from, triplet.object_predicate, triplet.object.indiv_value, true);
        if(used_solution.empty() == false)
          res_from.push_back(used_solution);
      }
    }
    else
    {
      // here we revert the problem since we know what we are looking for
      std::unordered_set<IndividualBranch*> candidates_indivs_on = getOn(index_indiv_from, triplet.object_predicate);
      if(std::find(candidates_indivs_on.begin(), candidates_indivs_on.end(), triplet.object.indiv_value) != candidates_indivs_on.end())
      {
        IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_indiv_from);
        IndivResult_t used_solution = checkInstantiatedTriplet(involved_indiv, triplet.object_predicate, triplet.object.indiv_value, true);
        if(used_solution.empty() == false)
          res_from.push_back(used_solution);
      }
    }
    return res_from;
  }

  std::vector<IndivResult_t> ReasonerRule::getFromData(RuleTriplet_t& triplet, const index_t& index_indiv_from)
  {
    std::vector<IndivResult_t> res_from;

    if(index_indiv_from == 0)
    {
      std::unordered_set<IndividualBranch*> candidates_indivs_from = getFrom(triplet.data_predicate, triplet.object.datatype_value->get());
      res_from.reserve(candidates_indivs_from.size());
      for(auto* indiv_from : candidates_indivs_from) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
      {
        IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from, triplet.data_predicate, triplet.object.datatype_value, true);
        if(used_solution.empty() == false)
          res_from.push_back(used_solution);
      }
    }
    else
    {
      // here we revert the problem since we know what we are looking for
      std::unordered_set<LiteralNode*> candidates_literals_on = getOn(index_indiv_from, triplet.data_predicate);
      if(std::find(candidates_literals_on.begin(), candidates_literals_on.end(), triplet.object.datatype_value) != candidates_literals_on.end())
      {
        IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_indiv_from);
        IndivResult_t used_solution = checkInstantiatedTriplet(involved_indiv, triplet.data_predicate, triplet.object.datatype_value, true);
        if(used_solution.empty() == false)
          res_from.push_back(used_solution);
      }
    }
    return res_from;
  }

  // returns IndivResults with either IndividualBranch or LiteralNode, explanation and triplets used
  std::vector<IndivResult_t> ReasonerRule::getOnObject(RuleTriplet_t& triplet, const index_t& index_indiv_on)
  {
    std::vector<IndivResult_t> res_on;

    if(index_indiv_on != 0)
    {
      // check if triplet.subject.indiv_value, predicate, indiv_on.indiv holds
      IndividualBranch* involved_indiv = ontology_->individual_graph_.findBranch(index_indiv_on);
      IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.object_predicate, involved_indiv, false);
      if(used_solution.empty() == false)
        res_on.push_back(used_solution);
      else
        return {};
    }
    else
    {
      // return all individuals matching the (triplet.subject.indiv_value, property, X)
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
    return res_on;
  }

  std::vector<IndivResult_t> ReasonerRule::getOnData(RuleTriplet_t& triplet, const index_t& index_literal_on)
  {
    std::vector<IndivResult_t> res;

    if(index_literal_on != 0)
    {
      LiteralNode* involved_literal = ontology_->data_property_graph_.createLiteral(LiteralNode::table.get(-index_literal_on));
      IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, involved_literal, false);
      if(used_solution.empty() == false)
        res.push_back(used_solution);
      else
        return {};
    }
    else
    {
      // return all literals matching the (triplet.subject.indiv_value, property, X)
      std::unordered_set<LiteralNode*> candidates_literals_on = getOn(triplet.subject.indiv_value->get(), triplet.data_predicate);
      if(candidates_literals_on.empty() == false)
      {
        res.reserve(candidates_literals_on.size());
        for(auto* literal_on : candidates_literals_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
        {
          IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, literal_on, false);
          if(used_solution.empty() == false)
            res.push_back(used_solution);
        }
      }
    }
    return res;
  }

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

              std::string explanation = indiv->same_as_[i].elem->value() + "|isA|" + class_selector->value();
              used_solution.explanations.emplace_back(explanation);

              used_solution.used_triplets.emplace_back(indiv->same_as_[i].elem->is_a_.has_induced_inheritance_relations[j],
                                                       indiv->same_as_[i].elem->is_a_.has_induced_object_relations[j],
                                                       indiv->same_as_[i].elem->is_a_.has_induced_data_relations[j]);

              if(indiv != indiv->same_as_[i].elem)
              {
                std::string explanation = indiv->value() + "|sameAs|" + indiv->same_as_[i].elem->value();
                used_solution.explanations.emplace_back(explanation);

                used_solution.used_triplets.emplace_back(indiv->same_as_.has_induced_inheritance_relations[i],
                                                         indiv->same_as_.has_induced_object_relations[i],
                                                         indiv->same_as_.has_induced_data_relations[i]);
              }

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
          std::string explanation = indiv->value() + "|isA|" + class_selector->value();

          used_solution.indiv = indiv;
          used_solution.explanations.emplace_back(explanation);

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