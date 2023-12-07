#include "ontologenius/core/reasoner/plugins/ReasonerAnonymous.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerAno::postReason()
{
  std::cout << "enter in ReasonerAno" << std::endl;
  std::lock_guard<std::shared_timed_mutex> lock(ontology_->individual_graph_.mutex_);
  std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.get();
  std::vector<AnonymousClassBranch_t*> anonyms = ontology_->anonymous_graph_.get();

  // TO UNCOMMENT
  // for(auto indiv : indivs)
  //   if((indiv->updated_ == true) || (indiv->flags_.find("equiv") != indiv->flags_.end()))
  //   {
  //     bool has_active_equiv = false;
  //     std::cout << "run ano on " << indiv->value() << std::endl;

  //     for(auto elem : anonyms)
  //       if(resolveTree(indiv, elem->ano_elem_))
  //         std::cout << "Individual " << indiv->value() << " is equivalent as " << elem->value() << " :" << elem->class_equiv_->value() << std::endl;
  //       else
  //         continue;

  //     if(has_active_equiv)
  //       indiv->flags_["equiv"] = {};
  //     else
  //       indiv->flags_.erase("equiv");
  //   }
}

bool ReasonerAno::resolveTree(LiteralNode* literal, AnonymousClassElement_t* ano_elem)
{ 
  if(ano_elem->logical_type_ == logical_and)
  {
    for(auto elem : ano_elem->sub_elements_)
      if(resolveTree(literal, elem) == false)
        return false;
    return true;
  }
  else if(ano_elem->logical_type_ == logical_or)
  {
    for(auto elem : ano_elem->sub_elements_)
      if(resolveTree(literal, elem))
        return true;
    return false;
  }
  else if(ano_elem->logical_type_ == logical_not)
    return !resolveTree(literal, ano_elem->sub_elements_.front());
  else
    return checkTypeRestriction(literal, ano_elem);

  return false;
}

bool ReasonerAno::relationExists(IndividualBranch_t* indiv_from, ObjectPropertyBranch_t* property, IndividualBranch_t* indiv_on)
{
  std::unordered_set<ObjectPropertyBranch_t*> down_properties;
  ontology_->object_property_graph_.getDownPtr(property, down_properties);

  for(auto& relation : indiv_from->object_relations_)
  {
    if(relation.second->get() == indiv_on->get())
    {
      if(down_properties.find(relation.first) != down_properties.end())
        return true;
    }
  }
  return false;
}

bool ReasonerAno::resolveTree(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->logical_type_ == logical_and)
  {
    for(auto elem : ano_elem->sub_elements_)
      if(resolveTree(indiv, elem) == false)
        return false;
    return true;
  }
  else if(ano_elem->logical_type_ == logical_or)
  {
    for(auto elem : ano_elem->sub_elements_)
      if(resolveTree(indiv, elem))
        return true;
    return false;
  }
  else if(ano_elem->logical_type_ == logical_not)
    return !resolveTree(indiv, ano_elem->sub_elements_.front());
  else
    return checkRestriction(indiv, ano_elem);

  return false;
}

bool ReasonerAno::checkRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->object_property_involved_ != nullptr || ano_elem->data_property_involved_ != nullptr)
    return checkCard(indiv, ano_elem);
  else if(ano_elem->class_involved_ != nullptr)
    return checkTypeRestriction(indiv, ano_elem);
  else if(ano_elem->individual_involved_ != nullptr)
    return checkIndividualRestriction(indiv, ano_elem);
  else if(ano_elem->oneof)
  {
    for(auto elem : ano_elem->sub_elements_)
      if(checkIndividualRestriction(indiv, elem) == true)
        return true;
  }
  return false;
}

bool ReasonerAno::checkTypeRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  return ontology_->individual_graph_.isA(indiv, ano_elem->class_involved_->get());
}

bool ReasonerAno::checkIndividualRestriction(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  return (std::find_if(indiv->same_as_.begin(), indiv->same_as_.end(), 
                      [elem = ano_elem->individual_involved_] (auto same) { return same == elem; }
                      ) != indiv->same_as_.end());
}

bool ReasonerAno::checkTypeRestriction(LiteralNode* literal, AnonymousClassElement_t* ano_elem)
{
  return (literal->type_ == ano_elem->card_.card_range_->type_);
}

bool ReasonerAno::checkCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  switch (ano_elem->card_.card_type_)
  {
    case CardType_t::cardinality_some:
      return checkSomeCard(indiv, ano_elem);
    case CardType_t::cardinality_min:
      return checkMinCard(indiv, ano_elem);
    case CardType_t::cardinality_max:
      return checkMaxCard(indiv, ano_elem);
    case CardType_t::cardinality_exactly:
      return checkExactlyCard(indiv, ano_elem);
    case CardType_t::cardinality_only:
      return checkOnlyCard(indiv, ano_elem);
    case CardType_t::cardinality_value:
      return checkValueCard(indiv, ano_elem);
    default:
      // Display::error();
      std::cout << "Cardinality type outside of [min, max, exactly, only, value, some] -> " << ano_elem->card_.card_type_ << std::endl;
      return false;
  }
}

bool ReasonerAno::checkMinCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->object_property_involved_ != nullptr)
    return checkMinCard(indiv->object_relations_.relations, ano_elem);
  else if(ano_elem->data_property_involved_ != nullptr)
    return checkMinCard(indiv->data_relations_.relations, ano_elem);
  else
    return false;
}

bool ReasonerAno::checkMaxCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->object_property_involved_ != nullptr)
    return checkMaxCard(indiv->object_relations_.relations, ano_elem);
  else if(ano_elem->data_property_involved_ != nullptr)
    return checkMaxCard(indiv->data_relations_.relations, ano_elem);
  else
    return false;
}

bool ReasonerAno::checkExactlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->object_property_involved_ != nullptr)
    return checkExactlyCard(indiv->object_relations_.relations, ano_elem);
  else if(ano_elem->data_property_involved_ != nullptr)
    return checkExactlyCard(indiv->data_relations_.relations, ano_elem);
  else
    return false;
}

bool ReasonerAno::checkOnlyCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->object_property_involved_ != nullptr)
    return checkOnlyCard(indiv->object_relations_.relations, ano_elem);
  else if(ano_elem->data_property_involved_ != nullptr)
    return checkOnlyCard(indiv->data_relations_.relations, ano_elem);
  else
    return false;
}

bool ReasonerAno::checkSomeCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->object_property_involved_ != nullptr)
    return checkSomeCard(indiv->object_relations_.relations, ano_elem);
  else if(ano_elem->data_property_involved_ != nullptr)
    return checkSomeCard(indiv->data_relations_.relations, ano_elem);
  else
    return false;
}

bool ReasonerAno::checkValueCard(IndividualBranch_t* indiv, AnonymousClassElement_t* ano_elem)
{
  return relationExists(indiv, ano_elem->object_property_involved_, ano_elem->individual_involved_);
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
