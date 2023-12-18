#include "ontologenius/core/reasoner/plugins/ReasonerAnonymous.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerAno::postReason()
{
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<InheritedRelationTriplets*> used;
   
  for(auto indiv : ontology_->individual_graph_.get())
  {
    if((indiv->updated_ == true) || (indiv->flags_.find("equiv") != indiv->flags_.end()))
    {
      bool has_active_equiv = false;
      for(auto anonymous : ontology_->anonymous_graph_.get())
      {
        used.clear();
        used.reserve(anonymous->depth_);
       
        if(checkClassesDisjointess(indiv, anonymous->class_equiv_) == false)
        {
          if(resolveTree(indiv, anonymous->ano_elem_, used))
          {
            std::cout << "Individual " << indiv->value() << " is equivalent to " << anonymous->value() << " :" << anonymous->class_equiv_->value() << std::endl;
            std::cout << "used vector size = " << used.size() << std::endl;

            if(ontology_->individual_graph_.conditionalPushBack(indiv->is_a_, ClassElement_t(anonymous->class_equiv_)))
              anonymous->class_equiv_->individual_childs_.emplace_back(indiv);
            
            for(auto induced_vector : used)
            { 
              if(induced_vector->exist(indiv, nullptr, anonymous->class_equiv_) == false)
              {
                induced_vector->push(indiv, nullptr, anonymous->class_equiv_);
                std::string explanation_reference = computeExplanation(used);
                explanations_.emplace_back("[ADD]" + indiv->value() + "|isA|" + anonymous->class_equiv_->value() + "|",
                                    "[ADD]" + explanation_reference);
              }
            }
          }
        }
        else
          std::cout << "disjointness error between classes of " + indiv->value() + " and " + anonymous->class_equiv_->value() << std::endl;
      }

      if(has_active_equiv)
        indiv->flags_["equiv"] = {};
      else
        indiv->flags_.erase("equiv");
    }
  }
}


std::string ReasonerAno::computeExplanation( std::vector<InheritedRelationTriplets*>& used)
{
  std::string explanation = "";

  return explanation;
}

bool ReasonerAno::checkClassesDisjointess(IndividualBranch_t* indiv, ClassBranch_t* class_equiv)
{
  std::unordered_set<ClassBranch_t*> disjoints;
  std::unordered_set<ClassBranch_t*> ups;

  // same_as impact ?
  for(auto is_a : indiv->is_a_)
  {
    disjoints.clear();
    ups.clear();
    ontology_->class_graph_.getUpPtr(is_a.elem, ups);
    ontology_->class_graph_.getDisjoint(class_equiv, disjoints);

    if(ontology_->class_graph_.firstIntersection(ups, disjoints) != nullptr)
      return true;
  }
  return false;
}

int ReasonerAno::relationExists(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  std::unordered_set<ObjectPropertyBranch_t*> down_properties;
  ontology_->object_property_graph_.getDownPtr(property, down_properties);

  for(size_t i = 0 ; i < indiv_from->object_relations_.size(); i++)
  {
    if(indiv_on->same_as_.size() > 0)
    {
      for(auto same_as_on : indiv_on->same_as_)
      {
        if(indiv_from->object_relations_[i].second->get() == same_as_on.elem->get())
        {
          if(down_properties.find(indiv_from->object_relations_[i].first) != down_properties.end())
            return i;
        }
      }
    }
    else if(indiv_from->object_relations_[i].second->get() == indiv_on->get())
    {
      if(down_properties.find(indiv_from->object_relations_[i].first) != down_properties.end())
        return i;
    }
  }
  return -1;

}

bool ReasonerAno::resolveTree(LiteralNode* literal, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
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

bool ReasonerAno::resolveTree(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
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
    return !resolveTree(indiv, ano_elem->sub_elements_.front(), used);
  }
  else
    return checkRestriction(indiv, ano_elem, used);

  return false;
}

bool ReasonerAno::checkRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  if(ano_elem->object_property_involved_ != nullptr || ano_elem->data_property_involved_ != nullptr)
    return checkCard(indiv, ano_elem, used);
  else if(ano_elem->class_involved_ != nullptr)
    return checkTypeRestriction(indiv, ano_elem, used);
  else if(ano_elem->oneof)
  {
    for(auto elem : ano_elem->sub_elements_)
      if(checkIndividualRestriction(indiv, elem) == true)
        return true;
  }
  used.clear();
  return false;
}

bool ReasonerAno::checkTypeRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  std::unordered_set<ClassBranch_t*> up;
  for(size_t i = 0 ; i < indiv->is_a_.size() ; i++)
  {
    up.clear();
    ontology_->class_graph_.getUpPtr(indiv->is_a_[i].elem, up);
    for(auto sub_elem : up)
    {
      if(sub_elem->get() == ano_elem->class_involved_->get())
      {
        used.push_back(&indiv->is_a_.has_induced_inheritance_relations[i]);
        return true;
      }
    }
  }
  used.clear();
  return false;

  // return (ontology_->individual_graph_.isA(indiv, ano_elem->class_involved_->get()))
}

bool ReasonerAno::checkTypeRestriction(LiteralNode* literal, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  return (literal->type_ == ano_elem->card_.card_range_->type_);
}

bool ReasonerAno::checkIndividualRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(indiv->same_as_.size() != 0)
  {
    return (std::find_if(indiv->same_as_.begin(), indiv->same_as_.end(), 
                      [elem = ano_elem->individual_involved_] (auto same) { return same == elem; }
                      ) != indiv->same_as_.end());
  }
  else
    return (indiv->get() == ano_elem->individual_involved_->get());
}

bool ReasonerAno::checkCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  switch (ano_elem->card_.card_type_)
  {
    case CardType_t::cardinality_some:
      return checkSomeCard(indiv, ano_elem, used);
    case CardType_t::cardinality_min:
      return checkMinCard(indiv, ano_elem, used);
    case CardType_t::cardinality_max:
      return checkMaxCard(indiv, ano_elem, used);
    case CardType_t::cardinality_exactly:
      return checkExactlyCard(indiv, ano_elem, used);
    case CardType_t::cardinality_only:
      return checkOnlyCard(indiv, ano_elem, used);
    case CardType_t::cardinality_value:
      return checkValueCard(indiv, ano_elem, used);
    default:
      std::cout << ("Cardinality type outside of [min, max, exactly, only, value, some] -> " + ano_elem->card_.card_type_) << std::endl;
      return false;
  }
}

bool ReasonerAno::checkMinCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  std::vector<size_t> indexes;
  if(ano_elem->object_property_involved_ != nullptr)
  {
    indexes = checkMinCard(indiv->object_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto index : indexes)
        used.push_back(&indiv->object_relations_.has_induced_inheritance_relations[index]);
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
      for(auto index : indexes)
        used.push_back(&indiv->data_relations_.has_induced_inheritance_relations[index]);
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

bool ReasonerAno::checkMaxCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  std::vector<size_t> indexes;
  if(ano_elem->object_property_involved_ != nullptr)
  {
    indexes = checkMaxCard(indiv->object_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto index : indexes)
        used.push_back(&indiv->object_relations_.has_induced_inheritance_relations[index]);
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
      for(auto index : indexes)
        used.push_back(&indiv->data_relations_.has_induced_inheritance_relations[index]);
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

bool ReasonerAno::checkExactlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  std::vector<size_t> indexes;
  if(ano_elem->object_property_involved_ != nullptr)
  {
    indexes = checkExactlyCard(indiv->object_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto index : indexes)
        used.push_back(&indiv->object_relations_.has_induced_inheritance_relations[index]);
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
      for(auto index : indexes)
        used.push_back(&indiv->data_relations_.has_induced_inheritance_relations[index]);
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

bool ReasonerAno::checkOnlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  std::vector<size_t> indexes;
  if(ano_elem->object_property_involved_ != nullptr)
  {
    indexes = checkOnlyCard(indiv->object_relations_.relations, ano_elem, used);
    if(indexes.size() > 0)
    {
      for(auto index : indexes)
        used.push_back(&indiv->object_relations_.has_induced_inheritance_relations[index]);
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
      for(auto index : indexes)
        used.push_back(&indiv->data_relations_.has_induced_inheritance_relations[index]);
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

bool ReasonerAno::checkSomeCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  int index;

  if(ano_elem->object_property_involved_ != nullptr)
  {
    index = checkSomeCard(indiv->object_relations_.relations, ano_elem, used);
    if(index != -1)
    {
      used.push_back(&indiv->object_relations_.has_induced_inheritance_relations[index]);
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
    if(index != -1)
    {
      used.push_back(&indiv->data_relations_.has_induced_inheritance_relations[index]);
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

bool ReasonerAno::checkValueCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem,  std::vector<InheritedRelationTriplets*>& used)
{
  int index = -1;
  if(indiv->same_as_.size() > 0)
  {
    for(auto& same_as : indiv->same_as_)
    {
      index = relationExists(same_as.elem, ano_elem->object_property_involved_, ano_elem->individual_involved_);
      if(index != -1)
        break;
    }
  }
  else
    index = relationExists(indiv, ano_elem->object_property_involved_, ano_elem->individual_involved_);

  if(index != -1)
  {
    used.push_back(&indiv->object_relations_.has_induced_inheritance_relations[index]);
    return true;
  }
  else
  {
    used.clear();
    return false;
  }
}

std::string ReasonerAno::getName()
{
  return "reasoner anonymous";
}

std::string ReasonerAno::getDescription()
{
  return "This reasoner resolves the anonymous classes i.e the equivalence relations.";
}

PLUGINLIB_EXPORT_CLASS(ReasonerAno, ReasonerInterface)

} // namespace ontologenius
