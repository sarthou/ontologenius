#include "ontologenius/core/reasoner/plugins/ReasonerAnonymous.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

ReasonerAnonymous::ReasonerAnonymous()
{
  standard_mode_ = false;
}

void ReasonerAnonymous::setParameter(const std::string& name, const std::string& value)
{
  if(name == "standard_mode" && value == "true")
    standard_mode_ = true;
}

void ReasonerAnonymous::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::shared_lock<std::shared_timed_mutex> lock_class(ontology_->class_graph_.mutex_);
  std::shared_lock<std::shared_timed_mutex> lock_prop(ontology_->object_property_graph_.mutex_);
  std::vector<std::pair<std::string, InheritedRelationTriplets*>> used;

  for(auto indiv : ontology_->individual_graph_.get())
  {
    if((indiv->updated_ == true) || (indiv->flags_.find("equiv") != indiv->flags_.end()) || indiv->hasUpdatedObjectRelation() || indiv->hasUpdatedDataRelation())
    {
      bool has_active_equiv = false;

      // Loop over every classes which includes equivalence relations
      for(auto anonymous : ontology_->anonymous_graph_.get())
      {
        bool tree_evaluation_result = false;
        bool is_already_a = std::any_of(indiv->is_a_.cbegin(), indiv->is_a_.cend(), [anonymous](const auto& is_a){ return is_a.elem == anonymous->class_equiv_; });

        if(is_already_a || checkClassesDisjointess(indiv, anonymous->class_equiv_) == false)
        {
          // Loop over every equivalence relations corresponding to one class
          for(auto anonymous_branch : anonymous->ano_elems_)
          {
            bool tree_first_layer_result = true;
            if(is_already_a == false)
              tree_first_layer_result = resolveFirstLayer(indiv, anonymous_branch);
            has_active_equiv = has_active_equiv || tree_first_layer_result;

            if(tree_first_layer_result == true)
            {
              used.clear();
              used.reserve(anonymous->depth_);
              tree_evaluation_result = resolveTree(indiv,  anonymous_branch, used);
            }
            
            if(has_active_equiv && tree_evaluation_result)
            {
              if(ontology_->individual_graph_.conditionalPushBack(indiv->is_a_, ClassElement_t(anonymous->class_equiv_, 1.0, true)))
              {
                if(ontology_->individual_graph_.conditionalPushBack(anonymous->class_equiv_->individual_childs_, IndividualElement_t(indiv, 1.0, true)))
                {
                  indiv->nb_updates_++;
                  anonymous->class_equiv_->nb_updates_++;
                  std::string explanation_reference = "";
                  
                  for(auto& induced_vector : used)
                  {
                    explanation_reference += induced_vector.first;
                    // check for nullptr because OneOf returns a (string, nullptr)
                    if(induced_vector.second != nullptr)
                    {
                      if(induced_vector.second->exist(indiv, nullptr, anonymous->class_equiv_) == false)
                      {
                        induced_vector.second->push(indiv, nullptr, anonymous->class_equiv_);
                        indiv->is_a_.relations.back().induced_traces.emplace_back(induced_vector.second);
                      }
                    }
                  }

                  nb_update_++;
                  explanations_.emplace_back("[ADD]" + indiv->value() + "|isA|" + anonymous->class_equiv_->value(),
                                             "[ADD]" + explanation_reference);
                } 
              }
              // once we get a valid equivalence for a class, we break out of the loop
              break;
            } 
          }
        }
        
        //Manages implicitly the NOT, MIN, MAX, EXACTLY cases
        if(tree_evaluation_result == false && anonymous->ano_elems_.size() != 0 && ontology_->individual_graph_.isA(indiv, anonymous->class_equiv_->get()) == true)
        {
          indiv->nb_updates_++;
          anonymous->class_equiv_->nb_updates_++;
          ontology_->individual_graph_.removeInheritage(indiv, anonymous->class_equiv_, explanations_, true);
        }
      }
      
      if(has_active_equiv)
        indiv->flags_["equiv"] = {};
      else
        indiv->flags_.erase("equiv");
    }
  }
}

bool ReasonerAnonymous::checkClassesDisjointess(IndividualBranch_t* indiv, ClassBranch_t* class_equiv)
{
  auto it = disjoints_cache_.find(class_equiv);
  std::unordered_set<ClassBranch_t*> disjoints;

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
  
  if(disjoints.size())
  {
    std::unordered_set<ClassBranch_t*> ups;
    ontology_->individual_graph_.getUpPtr(indiv, ups);
    return (ontology_->class_graph_.firstIntersection(ups, disjoints) != nullptr);
  }
  else
    return false;
}

int ReasonerAnonymous::relationExists(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  std::unordered_set<ObjectPropertyBranch_t*> down_properties;
  std::string explanation;
  ontology_->object_property_graph_.getDownPtr(property, down_properties);

  size_t relation_size = indiv_from->object_relations_.size();
  size_t same_size = indiv_on->same_as_.size();
  for(size_t i = 0 ; i < relation_size; i++)
  {
    for(size_t j = 0 ; j < same_size; j++)
    {
      if(indiv_from->object_relations_[i].second->get() == indiv_on->same_as_[j].elem->get())
      {
        if(down_properties.find(indiv_from->object_relations_[i].first) != down_properties.end())
        {
          explanation = indiv_on->value() + "|sameAs|" + indiv_on->same_as_[j].elem->value() + ";";
          used.emplace_back(explanation, indiv_on->same_as_.has_induced_inheritance_relations[j]);
          return i;
        }
      }
    }

    if(indiv_from->object_relations_[i].second->get() == indiv_on->get())
    {
      if(down_properties.find(indiv_from->object_relations_[i].first) != down_properties.end())
        return i;
    }
  }
  return -1;
}

bool ReasonerAnonymous::resolveFirstLayer(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->logical_type_ == logical_and)
    for(auto elem : ano_elem->sub_elements_)
      return resolveFirstLayer(indiv, elem);
  else if(ano_elem->logical_type_ == logical_or)
  {
    for(auto elem : ano_elem->sub_elements_)
      if(resolveFirstLayer(indiv, elem) == true)
        return true;
    return false;
  }
  else if(ano_elem->logical_type_ == logical_not)
    return !resolveFirstLayer(indiv, ano_elem->sub_elements_.front());
  else
    return checkRestrictionFirstLayer(indiv, ano_elem);
  return false;
}

bool ReasonerAnonymous::resolveTree(LiteralNode* literal, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{ 
  if(ano_elem->logical_type_ == logical_and)
  {
    for(auto elem : ano_elem->sub_elements_)
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
    for(auto elem : ano_elem->sub_elements_)
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

bool ReasonerAnonymous::resolveTree(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  if(ano_elem->logical_type_ == logical_and)
  {
    for(auto elem : ano_elem->sub_elements_)
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
    for(auto elem : ano_elem->sub_elements_)
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
    if(standard_mode_== true)
      return false;
    else
      return !resolveTree(indiv, ano_elem->sub_elements_.front(), used);
  }
  else
    return checkRestriction(indiv, ano_elem, used);

  return false;
}

bool ReasonerAnonymous::checkRestrictionFirstLayer(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->object_property_involved_ != nullptr)
  {
    if(indiv->same_as_.size() != 0)
    {
      for(auto& indiv_same : indiv->same_as_)
      {
        if(checkPropertyExistence(indiv_same.elem->object_relations_.relations, ano_elem))
          return true;
      }
    }
    return checkPropertyExistence(indiv->object_relations_.relations, ano_elem);
  }
  else if (ano_elem->data_property_involved_ != nullptr)
  {
    if(indiv->same_as_.size() != 0)
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
    for(auto indiv_elem : ano_elem->sub_elements_)
    {
      if(indiv->same_as_.size() != 0)
      {
        if(std::find_if(indiv->same_as_.begin(), indiv->same_as_.end(), 
                      [elem = indiv_elem->individual_involved_] (auto same) { return same == elem; }
                      ) != indiv->same_as_.end())
                      { return true;}
      }
      if(indiv->get() == indiv_elem->individual_involved_->get())
        return true;
    }
  }
  return false;
}

bool ReasonerAnonymous::checkRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  std::string explanation;

  if(ano_elem->object_property_involved_ != nullptr || ano_elem->data_property_involved_ != nullptr)
  {
    if(indiv->same_as_.size() != 0)
    {
      size_t same_size = indiv->same_as_.size();
      for(size_t i = 0 ; i < same_size; i++)
      {
        if(indiv->same_as_[i].elem->get() != indiv->get()) 
        {
          if(checkCard(indiv->same_as_[i].elem, ano_elem, used))
          {
            explanation = indiv->value() + "|sameAs|" + indiv->same_as_[i].elem->value() + ";";
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
    for(auto elem : ano_elem->sub_elements_)
    {
      if(one_of != "")
        one_of += ", ";
      one_of += elem->individual_involved_->value();
      if(checkIndividualRestriction(indiv, elem, used) == true)
        explanation = indiv->value();
    }
    if(explanation != "")
    {
      explanation += "|isOneOf|(" + one_of +")";
      used.emplace_back(explanation, nullptr);
      return true;
    }
  }
  
  used.clear();
  return false;
}

bool ReasonerAnonymous::checkTypeRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  std::string explanation;

  if(indiv->same_as_.size() > 0)
  {
    size_t same_size = indiv->same_as_.size();
    for(size_t i = 0 ; i < same_size; i++)
    {
      if(indiv->same_as_[i].elem->get() != indiv->get())
      {
        size_t is_a_size = indiv->same_as_[i].elem->is_a_.size();
        for(size_t j = 0; j < is_a_size; j++)
        {
          if(existInInheritance(indiv->same_as_[i].elem->is_a_[j].elem, ano_elem->class_involved_->get(), used))
          {
            explanation = indiv->same_as_[i].elem->value() + "|isA|" + ano_elem->class_involved_->value() + ";";
            used.emplace_back(explanation, indiv->same_as_[i].elem->is_a_.has_induced_inheritance_relations[j]);

            explanation = indiv->value() + "|sameAs|" + indiv->same_as_[i].elem->value() + ";";
            used.emplace_back(explanation, indiv->same_as_.has_induced_inheritance_relations[i]);
            return true;
          }
        }
      }
    }
  }
  else
  {
    size_t is_a_size = indiv->is_a_.size();
    for(size_t i = 0 ; i < is_a_size; i++)
    {
      if(existInInheritance(indiv->is_a_[i].elem, ano_elem->class_involved_->get(), used))
      {
        explanation = indiv->value() + "|isA|" + ano_elem->class_involved_->value() + ";";
        used.emplace_back(explanation, indiv->is_a_.has_induced_inheritance_relations[i]);
        return true;
      }
    }
  }

  used.clear();
  return false;
}

bool ReasonerAnonymous::checkTypeRestriction(LiteralNode* literal, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  // To prevent warning of unused parameter for used
  (void)used;
  return (literal->type_ == ano_elem->card_.card_range_->type_);
}

bool ReasonerAnonymous::checkIndividualRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem, std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  std::string explanation;

  if(indiv->same_as_.size() != 0)
  {
    size_t same_size = indiv->same_as_.size();
    for(size_t i = 0 ; i < same_size; i++)
    {
      if(indiv->same_as_[i].elem->get() != indiv->get())
      {
        if(indiv->same_as_[i].elem->get() == ano_elem->individual_involved_->get())
        {
          explanation = indiv->value() + "|sameAs|" + ano_elem->individual_involved_->value() + ";";
          used.emplace_back(explanation, indiv->same_as_.has_induced_inheritance_relations[i]);
          return true;
        }  
      }  
    }
  }

  return (indiv->get() == ano_elem->individual_involved_->get());
}

bool ReasonerAnonymous::checkCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  switch (ano_elem->card_.card_type_)
  {
    case CardType_t::cardinality_some:
      return checkSomeCard(indiv, ano_elem, used);
    case CardType_t::cardinality_min:
      if(standard_mode_== true)
        return false;
      else
        return checkMinCard(indiv, ano_elem, used);
    case CardType_t::cardinality_max:
      if(standard_mode_== true)
        return false;
      else
        return checkMaxCard(indiv, ano_elem, used);
    case CardType_t::cardinality_exactly:
      if(standard_mode_== true)
        return false;
      else
        return checkExactlyCard(indiv, ano_elem, used);
    case CardType_t::cardinality_only:
      if(standard_mode_== true)
        return false;
      else
        return checkOnlyCard(indiv, ano_elem, used);
    case CardType_t::cardinality_value:
      return checkValueCard(indiv, ano_elem, used);
    default:
      std::cout << ("Cardinality type outside of [min, max, exactly, only, value, some] -> " + ano_elem->card_.card_type_) << std::endl;
      return false;
  }
}

bool ReasonerAnonymous::checkMinCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  std::vector<std::pair<std::string, size_t>> indexes;

  if(ano_elem->object_property_involved_ != nullptr)
  {
    indexes = checkMinCard(indiv->object_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto& index : indexes)
        used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
      return true;
    }
    else
    {
      used.clear();
      return false;
    }
  }
  else if(ano_elem->data_property_involved_ != nullptr)
  {
    indexes = checkMinCard(indiv->data_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto& index : indexes)
        used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
      return true;
    }
    else
    {
      used.clear();
      return false;
    }
  }
  else
  {
    used.clear();
    return false;
  }
}

bool ReasonerAnonymous::checkMaxCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  std::vector<std::pair<std::string, size_t>> indexes;

  if(ano_elem->object_property_involved_ != nullptr)
  {
    indexes = checkMaxCard(indiv->object_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto& index : indexes)
        used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
      return true;
    }
    else
    {
      used.clear();
      return false;
    }
  }
  else if(ano_elem->data_property_involved_ != nullptr)
  {
    indexes = checkMaxCard(indiv->data_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto& index : indexes)
        used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
      return true;
    }
    else
    {
      used.clear();
      return false;
    }
  }
  else
  {
    used.clear();
    return false;
  }
}

bool ReasonerAnonymous::checkExactlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  std::vector<std::pair<std::string, size_t>> indexes;

  if(ano_elem->object_property_involved_ != nullptr)
  {
    indexes = checkExactlyCard(indiv->object_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto& index : indexes)
        used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
      return true;
    }
    else
    {
      used.clear();
      return false;
    }
  }
  else if(ano_elem->data_property_involved_ != nullptr)
  {
    indexes = checkExactlyCard(indiv->data_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto& index : indexes)
        used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
      return true;
    }
    else
    {
      used.clear();
      return false;
    }
  }
  else
  {
    used.clear();
    return false;
  }
}

bool ReasonerAnonymous::checkOnlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  std::vector<std::pair<std::string, size_t>> indexes;

  if(ano_elem->object_property_involved_ != nullptr)
  {
    indexes = checkOnlyCard(indiv->object_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto& index : indexes)
        used.emplace_back(indiv->value() + "|" + index.first, indiv->object_relations_.has_induced_inheritance_relations[index.second]);
      return true;
    }
    else
    {
      used.clear();
      return false;
    }
  }
  else if(ano_elem->data_property_involved_ != nullptr)
  {
    indexes = checkOnlyCard(indiv->data_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto& index : indexes)
        used.emplace_back(indiv->value() + "|" + index.first, indiv->data_relations_.has_induced_inheritance_relations[index.second]);
      return true;
    }
    else
    {
      used.clear();
      return false;
    }
  }
  used.clear();
  return false;
}

bool ReasonerAnonymous::checkSomeCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
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
    else
    {
      used.clear();
      return false;
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
    else
    {
      used.clear();
      return false;
    }
  }
  else
  {
    used.clear();
    return false;
  }
}

bool ReasonerAnonymous::checkValueCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<std::pair<std::string, InheritedRelationTriplets*>>& used)
{
  int index = -1;
  std::string explanation;

  index = relationExists(indiv, ano_elem->object_property_involved_, ano_elem->individual_involved_, used);
  
  if(index != -1)
  {
    explanation = indiv->value() + "|" + ano_elem->object_property_involved_->value() + "|" + ano_elem->individual_involved_->value() + ";";
    used.emplace_back(explanation , indiv->object_relations_.has_induced_inheritance_relations[index]);
    return true;
  }
  else
  {
    used.clear();
    return false;
  }
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