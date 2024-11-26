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
      std::vector<RuleResult_t> results; // contains every valid instantiations
      // transform in here each vect of vect into RuleResult
      std::vector<std::vector<IndivResult_t>> results_resolve;
      std::vector<IndivResult_t> empty_accu(rule_branch->to_variables_.size(), IndivResult_t());
      results_resolve = resolve(rule_branch, rule_branch->rule_body_, empty_accu);

      // if(results_resolve.size() > 0)
      //   std::cout << "size of results : " << results_resolve.size() << ", size of 1st result : " << results_resolve.front().size() << std::endl;

      // transform results into RuleResults
      // std::vector<RuleResult_t> final_results = transformResults(results_resolve);
    }
  }

  // std::vector<RuleResult_t> ReasonerRule::resolve(RuleBranch* rule_branch, std::vector<RuleTriplet_t>& atoms, const std::vector<IndivResult_t>& prev_res)
  // {
  //   std::vector<RuleResult_t> res;

  //   return res;
  // }

  std::vector<RuleResult_t> ReasonerRule::transformResults(std::vector<std::vector<IndivResult_t>>& results)
  {
    std::vector<RuleResult_t> final_res;
    final_res.reserve(results.size());
    // final_res.reserve(values.size());
    for(auto& elem : results)
    {
      RuleResult_t new_res;
      for(auto& res : elem)
      {
        new_res.indivs.push_back(res.indiv);
        // std::cout << new_res.toString() << std::endl;
        new_res.explanations.push_back(res.explanations);
        new_res.triplets_used.push_back(res.used_triplets);
      }

      final_res.push_back(new_res);
    }
  }

  std::vector<std::vector<IndivResult_t>> ReasonerRule::resolve(RuleBranch* rule_branch, std::vector<RuleTriplet_t>& atoms, std::vector<IndivResult_t>& accu)
  {
    std::vector<IndivResult_t> values; // will contain every IndividualBranch* matching the Atom Expression
    int64_t var_index = 0;
    resolveAtom(rule_branch, atoms.front(), accu, var_index, values); // returns the individuals matching the triplet (with explanations and used_relations)

    if(values.empty() == true)
      return {};

    if(atoms.size() > 1) // means that we haven't reached the end of the rule antecedents
    {
      std::vector<RuleTriplet_t> new_atoms(atoms.begin() + 1, atoms.end()); // move on to the next atom

      std::vector<std::vector<IndivResult_t>> res; // or std::vector<RuleResult>
      res.reserve(values.size());                  // the size of values is the number of individuals which matched the previously evaluated atom
      std::vector<IndivResult_t> new_accu(accu);   // create a copy of the state of the accu (instantiated variables so far)

      for(auto& value : values)
      {
        // TO MERGE IN ONE CONDITION
        if(accu[var_index].indiv != nullptr) // if no individual was stored in the accu (no instantiation has been done for that variable)
        {
          if(accu[var_index].indiv != value.indiv) // if the individual in the accu and the one found are different, then we continue
            continue;
        }
        else
          new_accu[var_index] = value;

        if(accu[var_index].literal != nullptr)
        {
          if(accu[var_index].literal != value.literal)
            continue;
        }
        else
          new_accu[var_index] = value;

        std::vector<std::vector<IndivResult_t>> local_res = resolve(rule_branch, new_atoms, new_accu); // new solutions updated with the new_accu
        if(local_res.empty() == false)                                                                 // no solution has been found with the already assigned variables
        {
          for(auto& lr : local_res)
          {
            lr[var_index] = value;
            res.push_back(std::move(lr)); // move the found new instantiation into the res vector
          }
        }
      }
      return res;
    }
    else // we reached the end of an exploration, we backpropagate each instantiated variable into the resulting vector
    {
      // here we are supposed to have one indiv/literal for each atom (hasObject(ndiv1, indiv2) -> 2 indivs / has_data(indiv1, boolean#true) ->  indiv and 1 literal)
      std::vector<std::vector<IndivResult_t>> res(values.size(), std::vector<IndivResult_t>(rule_branch->variables_.size()));
      std::cout << "body size : " << rule_branch->rule_body_.size() << std::endl;
      // std::vector<std::vector<IndivResult_t>> res(values.size());
      std::cout << "variables size : " << rule_branch->variables_.size() << std::endl;
      std::cout << "values size : " << values.size() << std::endl;
      size_t cpt = 0;
      for(auto& value : values)
        res[cpt++][var_index] = value;
      return res;
    }
  }

  void ReasonerRule::resolveAtom(RuleBranch* rule_branch, RuleTriplet_t triplet, std::vector<IndivResult_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    std::cout << " ============  Evaluating triplet : " << triplet.toString() << "==============" << std::endl;
    switch(triplet.atom_type_)
    {
    case class_atom:
      resolveClassAtom(rule_branch, triplet, accu, var_index, values);
      break;
    case object_atom:
      resolveObjectAtom(rule_branch, triplet, accu, var_index, values);
      break;
    case data_atom:
      resolveDataAtom(rule_branch, triplet, accu, var_index, values);
      break;
    case builtin_atom:
      /* code */
      break;

    default:
      break;
    }
  }

  void ReasonerRule::resolveClassAtom(RuleBranch* rule_branch, RuleTriplet_t triplet, std::vector<IndivResult_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
  {
    var_index = triplet.subject.variable_id;
    if(triplet.subject.is_variable == true)
    {
      if(accu[var_index].indiv == nullptr) // has no previous value
        values = getType(triplet.class_predicate);
      else // has a previous value, so we evaluate it
      {
        if(ontology_->individual_graph_.isA(accu[var_index].indiv, triplet.class_predicate->value()) == true)
        {
          IndivResult_t used = checkInstantiatedTriplet(accu[var_index].indiv, triplet.class_predicate);
          values.push_back(used);
        }
      }
    }
    else // atom is not variable, so we check the individual in the triplet
    {
      if(ontology_->individual_graph_.isA(triplet.subject.indiv_value, triplet.class_predicate->value()) == true)
      {
        IndivResult_t used = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.class_predicate);
        values.push_back(used);
      }
    }
    // need to take into account the case in which a instantiated Class Atom can never be resolved so that we exit from the rule
  }

  void ReasonerRule::resolveObjectAtom(RuleBranch* rule_branch, RuleTriplet_t triplet, std::vector<IndivResult_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
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
      if(accu[var_index].indiv != nullptr)
      {
        triplet.subject.indiv_value = accu[var_index].indiv;
        var_index = triplet.object.variable_id;
        values = getOnObject(triplet, accu[var_index]);
      }
      else
      {
        if(accu[triplet.object.variable_id].indiv != nullptr)
        {
          triplet.object.indiv_value = accu[triplet.object.variable_id].indiv;
          IndivResult_t none_value; // equivalent of the getDefaultSelector()
          values = getFromObject(triplet, none_value);
        }
        else
          std::cout << "no variable bounded to any of the variable fields" << std::endl;
      }
    }
  }

  void ReasonerRule::resolveDataAtom(RuleBranch* rule_branch, RuleTriplet_t triplet, std::vector<IndivResult_t>& accu, int64_t& var_index, std::vector<IndivResult_t>& values)
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
      if(accu[var_index].indiv != nullptr)
      {
        triplet.subject.indiv_value = accu[var_index].indiv;
        var_index = triplet.object.variable_id;
        values = getOnData(triplet, accu[var_index]);
      }
      else
      {
        if(accu[triplet.object.variable_id].literal != nullptr)
        {
          triplet.object.datatype_value = accu[triplet.object.variable_id].literal;
          IndivResult_t none_value;
          values = getFromData(triplet, none_value);
        }
        else
          std::cout << "no variable bounded to any of the variable fields" << std::endl;
      }
    }
  }

  // std::vector<IndivResult_t> ReasonerRule::getOnResults(IndividualBranch* indiv_from, ObjectPropertyBranch* predicate)
  // {
  //   std::unordered_set<IndividualBranch*> indivs_on;
  //   std::vector<IndivResult_t> values;

  //   indivs_on = getOn(indiv_from, predicate); // check if some indiv match the atom
  //   values.reserve(indivs_on.size());

  //   for(auto* indiv_on : indivs_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
  //   {
  //     IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from, predicate, indiv_on, false);

  //     if(used_solution.empty() == false)
  //       values.push_back(used_solution);
  //   }
  //   return values;
  // }

  // std::vector<IndivResult_t> ReasonerRule::getOnResults(IndividualBranch* indiv_from, DataPropertyBranch* predicate)
  // {
  //   std::unordered_set<LiteralNode*> literals_on;
  //   std::vector<IndivResult_t> values;

  //   literals_on = getOn(indiv_from, predicate); // check if some indiv match the atom

  //   values.reserve(literals_on.size());

  //   for(auto* literal_on : literals_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
  //   {
  //     IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from, predicate, literal_on, false);

  //     if(used_solution.empty() == false)
  //       values.push_back(used_solution);
  //   }
  //   return values;
  // }

  // returns IndivResults with IndividualBranch, explanation and triplets used
  std::vector<IndivResult_t> ReasonerRule::getFromObject(RuleTriplet_t& triplet, IndivResult_t& indiv_from) // here T can be either Indiv or Literal
  {
    std::vector<IndivResult_t> res_from;

    if(indiv_from.indiv == nullptr)
    {
      std::unordered_set<IndividualBranch*> candidates_indivs_from = getFrom(triplet.object_predicate, triplet.object.indiv_value);
      res_from.reserve(candidates_indivs_from.size());
      for(auto* indiv_from : candidates_indivs_from) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
      {
        IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from, triplet.object_predicate, triplet.object.indiv_value, true);
        if(used_solution.empty() == false) // normally it never should be empty since we arleady checked that getOnInvidivuals returned some
          res_from.push_back(used_solution);
      }
    }
    else
    {
      // here we revert the problem since we know what we are looking for
      std::unordered_set<IndividualBranch*> candidates_indivs_on = getOn(indiv_from.indiv, triplet.object_predicate);
      if(std::find(candidates_indivs_on.begin(), candidates_indivs_on.end(), triplet.object.indiv_value) != candidates_indivs_on.end())
      {
        IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from.indiv, triplet.object_predicate, triplet.object.indiv_value, true);
        if(used_solution.empty() == false) // normally it never should be empty since we arleady checked that getOnInvidivuals returned some
          res_from.push_back(used_solution);
      }
    }
    return res_from;
  }

  std::vector<IndivResult_t> ReasonerRule::getFromData(RuleTriplet_t& triplet, IndivResult_t& indiv_from)
  {
    std::vector<IndivResult_t> res_from;

    if(indiv_from.indiv == nullptr)
    {
      std::unordered_set<IndividualBranch*> candidates_indivs_from = getFrom(triplet.data_predicate, triplet.object.datatype_value);
      res_from.reserve(candidates_indivs_from.size());
      for(auto* indiv_from : candidates_indivs_from) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
      {
        IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from, triplet.data_predicate, triplet.object.datatype_value, true);
        if(used_solution.empty() == false) // normally it never should be empty since we arleady checked that getOnInvidivuals returned some
          res_from.push_back(used_solution);
      }
    }
    else
    {
      // here we revert the problem since we know what we are looking for
      std::unordered_set<LiteralNode*> candidates_literals_on = getOn(indiv_from.indiv, triplet.data_predicate);
      if(std::find(candidates_literals_on.begin(), candidates_literals_on.end(), triplet.object.datatype_value) != candidates_literals_on.end())
      {
        IndivResult_t used_solution = checkInstantiatedTriplet(indiv_from.indiv, triplet.data_predicate, triplet.object.datatype_value, true);
        if(used_solution.empty() == false) // normally it never should be empty since we arleady checked that getOnInvidivuals returned some
          res_from.push_back(used_solution);
      }
    }
    return res_from;
  }

  // returns IndivResults with either IndividualBranch or LiteralNode, explanation and triplets used
  std::vector<IndivResult_t> ReasonerRule::getOnObject(RuleTriplet_t& triplet, IndivResult_t& indiv_on)
  {
    std::vector<IndivResult_t> res_on;

    if(indiv_on.indiv != nullptr)
    {
      // check if triplet.subject.indiv_value, predicate, indiv_on.indiv holds
      IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.object_predicate, indiv_on.indiv, false);
      if(used_solution.empty() == false)
        res_on.push_back(used_solution);
      else
        return {};
    }
    else if(indiv_on.indiv == nullptr)
    {
      // return all individuals matching the (triplet.subject.indiv_value, property, X)
      std::unordered_set<IndividualBranch*> candidates_indivs_on = getOn(triplet.subject.indiv_value, triplet.object_predicate);
      if(candidates_indivs_on.empty() == false)
      {
        res_on.reserve(candidates_indivs_on.size());
        for(auto* indiv_on : candidates_indivs_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
        {
          IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.object_predicate, indiv_on, false);
          if(used_solution.empty() == false) // normally it never should be empty since we arleady checked that getOnInvidivuals returned some
            res_on.push_back(used_solution);
        }
      }
    }
    return res_on;
  }

  std::vector<IndivResult_t> ReasonerRule::getOnData(RuleTriplet_t& triplet, IndivResult_t& literal_on)
  {
    std::vector<IndivResult_t> res;

    if(literal_on.literal != nullptr)
    {
      IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, literal_on.literal, false);
      if(used_solution.empty() == false)
        res.push_back(used_solution);
      else
        return {};
    }
    else if(literal_on.literal == nullptr)
    {
      // return all literals matching the (triplet.subject.indiv_value, property, X)
      std::unordered_set<LiteralNode*> candidates_literals_on = getOn(triplet.subject.indiv_value, triplet.data_predicate);
      if(candidates_literals_on.empty() == false)
      {
        res.reserve(candidates_literals_on.size());
        for(auto* literal_on : candidates_literals_on) // here we have to explore based on the selected individuals to compute explanations and memory adress to store into values
        {
          IndivResult_t used_solution = checkInstantiatedTriplet(triplet.subject.indiv_value, triplet.data_predicate, literal_on, false);
          if(used_solution.empty() == false) // normally it never should be empty since we arleady checked that getOnInvidivuals returned some
            res.push_back(used_solution);
        }
      }
    }
    return res;
  }

  // returns IndividualBranch without explanations
  std::unordered_set<IndividualBranch*> ReasonerRule::getFrom(ObjectPropertyBranch* property, IndividualBranch* individual_on)
  {
    std::unordered_set<IndividualBranch*> indivs_from;
    index_t indiv_index = individual_on->get();

    const std::unordered_set<index_t> object_properties = ontology_->object_property_graph_.getDownId(property->get());

    for(auto& indiv_i : ontology_->individual_graph_.all_branchs_)
      for(const IndivObjectRelationElement& relation : indiv_i->object_relations_)
        for(const index_t id : object_properties)
          if(relation.first->get() == id)
          {
            if(relation.second->get() == indiv_index)
            {
              indivs_from.insert(indiv_i);
              break;
            }
          }

    return indivs_from;
  }

  std::unordered_set<IndividualBranch*> ReasonerRule::getFrom(DataPropertyBranch* property, LiteralNode* literal_on)
  {
    std::unordered_set<IndividualBranch*> indivs_from;
    index_t literal_index = literal_on->get();

    const std::unordered_set<index_t> data_properties = ontology_->data_property_graph_.getDownId(property->get());

    for(auto& indiv_i : ontology_->individual_graph_.all_branchs_)
      for(const IndivDataRelationElement& relation : indiv_i->data_relations_)
        for(const index_t id : data_properties)
          if(relation.first->get() == id)
          {
            if(relation.second->get() == literal_index)
            {
              indivs_from.insert(indiv_i);
              break;
            }
          }

    return indivs_from;
  }

  // returns either IndividualBranch or Literalode without explanations
  std::unordered_set<IndividualBranch*> ReasonerRule::getOn(IndividualBranch* indiv_from, ObjectPropertyBranch* predicate)
  {
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

  std::unordered_set<LiteralNode*> ReasonerRule::getOn(IndividualBranch* indiv_from, DataPropertyBranch* predicate)
  {
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
    std::vector<IndivResult_t> indiv_of_class;
    std::unordered_set<IndividualBranch*> indiv_res;

    indiv_res = ontology_->individual_graph_.getType(class_selector);
    indiv_of_class.reserve(indiv_res.size());

    for(auto* indiv : indiv_res)
      indiv_of_class.push_back(checkInstantiatedTriplet(indiv, class_selector));

    return indiv_of_class;
  }

  // check the existence of the triplet and compute the explanation
  IndivResult_t ReasonerRule::checkInstantiatedTriplet(IndividualBranch* indiv, ClassBranch* class_selector)
  {
    std::string explanation;
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

              explanation = indiv->same_as_[i].elem->value() + "|isA|" + class_selector->value() + ";";
              used_solution.explanations.emplace_back(explanation);

              RuleUsedTriplet_t used_same(indiv->same_as_[i].elem->is_a_.has_induced_inheritance_relations[j],
                                          indiv->same_as_[i].elem->is_a_.has_induced_object_relations[j],
                                          indiv->same_as_[i].elem->is_a_.has_induced_data_relations[j]);

              // IndivResult_t used(indiv, explanation, used_same); // new version with constructor

              used_solution.used_triplets.emplace_back(used_same);

              explanation = indiv->value() + "|sameAs|" + indiv->same_as_[i].elem->value() + ";";
              used_solution.explanations.emplace_back(explanation);

              RuleUsedTriplet_t used_indiv(indiv->same_as_.has_induced_inheritance_relations[i],
                                           indiv->same_as_.has_induced_object_relations[i],
                                           indiv->same_as_.has_induced_data_relations[i]);
              // IndivResult_t used_2(indiv, explanation, used_same);

              used_solution.used_triplets.emplace_back(used_indiv);

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
          IndivResult_t used_solution;
          explanation = indiv->value() + "|isA|" + class_selector->value() + ";";

          used_solution.indiv = indiv;
          used_solution.explanations.emplace_back(explanation);

          RuleUsedTriplet_t used_isa(indiv->is_a_.has_induced_inheritance_relations[i],
                                     indiv->is_a_.has_induced_object_relations[i],
                                     indiv->is_a_.has_induced_data_relations[i]);
          // IndivResult_t used(indiv, explanation, used_isa); // new version with constructor
          used_solution.used_triplets.emplace_back(used_isa);

          return used_solution;
        }
      }
    }
    return IndivResult_t();
  }

  IndivResult_t ReasonerRule::checkInstantiatedTriplet(IndividualBranch* indiv_from, ObjectPropertyBranch* property_predicate, IndividualBranch* indiv_on, bool var_from)
  {
    IndivResult_t used_solution;
    int index = -1;

    index = checkInstantiatedTriplet(indiv_from, property_predicate, indiv_on, indiv_from->object_relations_.relations, used_solution);

    if(index != -1)
    {
      // if(var_from == true)
      //   used_solution.indiv = indiv_from; // mark the indiv used in this atom evaluation -> maybe useless
      // else
      //   used_solution.indiv = indiv_on; // mark the indiv used in this atom evaluation
      used_solution.indiv = indiv_from;

      RuleUsedTriplet_t used_object(indiv_from->object_relations_.has_induced_inheritance_relations[index],
                                    indiv_from->object_relations_.has_induced_object_relations[index],
                                    indiv_from->object_relations_.has_induced_data_relations[index]);
      used_solution.used_triplets.emplace_back(used_object);
    }

    return used_solution;
  }

  IndivResult_t ReasonerRule::checkInstantiatedTriplet(IndividualBranch* indiv_from, DataPropertyBranch* property_predicate, LiteralNode* literal_on, bool var_from) // if var_from = true, then used_solution used indiv_from, else it used resource_on
  {
    IndivResult_t used_solution;
    int index = -1;

    index = checkInstantiatedTriplet(indiv_from, property_predicate, literal_on, indiv_from->data_relations_.relations, used_solution);

    if(index != -1)
    {
      // if(var_from == true)
      //   used_solution.indiv = indiv_from; // mark the indiv used in this atom evaluation
      // else
      //   used_solution.literal = literal_on; // mark the literal used in this atom evaluation
      used_solution.indiv = indiv_from;

      RuleUsedTriplet_t used_data(indiv_from->data_relations_.has_induced_inheritance_relations[index],
                                  indiv_from->data_relations_.has_induced_object_relations[index],
                                  indiv_from->data_relations_.has_induced_data_relations[index]);
      used_solution.used_triplets.emplace_back(used_data);
    }

    return used_solution;
  }

  // check the match between either two individuals or literals
  bool ReasonerRule::checkValue(IndividualBranch* indiv_from, IndividualBranch* indiv_on, IndivResult_t& used)
  {
    std::string explanation;

    if(indiv_from->same_as_.empty() == false)
    {
      const size_t same_size = indiv_from->same_as_.size();
      for(size_t j = 0; j < same_size; j++)
      {
        if(indiv_from->same_as_[j].elem->get() == indiv_on->get())
        {
          explanation = indiv_from->value() + "|sameAs|" + indiv_on->value() + ";";

          used.explanations.emplace_back(explanation);

          RuleUsedTriplet_t used_elem(indiv_from->same_as_.has_induced_inheritance_relations[j],
                                      indiv_from->same_as_.has_induced_object_relations[j],
                                      indiv_from->same_as_.has_induced_data_relations[j]);

          used.used_triplets.emplace_back(used_elem);

          return true;
        }
      }
    }
    else
    {
      if(indiv_from->get() == indiv_on->get())
        return true;
    }

    return false;
  }

  bool ReasonerRule::checkValue(LiteralNode* literal_from, LiteralNode* literal_on, IndivResult_t& used)
  {
    (void)used;

    if(literal_from->get() == literal_on->get())
      return true;

    return false;
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