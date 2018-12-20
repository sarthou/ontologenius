#include "ontoloGenius/core/ontologyOperators/differenceFinder.h"

#include "ontoloGenius/core/ontoGraphs/Ontology.h"

std::vector<std::string> differenceFinder::getDiff(Ontology* onto1, Ontology* onto2, const std::string& concept)
{
  comparator_t comp1, comp2;

  IndividualBranch_t* indiv_onto1 = onto1->individual_graph_.findBranch(concept);
  IndividualBranch_t* indiv_onto2 = onto2->individual_graph_.findBranch(concept);

  if((indiv_onto1 == nullptr) && (indiv_onto2 == nullptr))
  {
    ClassBranch_t* class_onto1 = onto1->class_graph_.findBranch(concept);
    ClassBranch_t* class_onto2 = onto2->class_graph_.findBranch(concept);
    if((class_onto1 == nullptr) && (class_onto2 != nullptr))
      comp2 = toComparator(class_onto2);
    else if((class_onto1 != nullptr) && (class_onto2 == nullptr))
      comp1 = toComparator(class_onto1);
    else if((class_onto1 != nullptr) && (class_onto2 != nullptr))
    {
      comp1 = toComparator(class_onto1);
      comp2 = toComparator(class_onto2);
    }
  }
  else if(indiv_onto1 == nullptr)
  {
    comp2 = toComparator(indiv_onto2);
    ClassBranch_t* class_onto1 = onto1->class_graph_.findBranch(concept);
    if(class_onto1 != nullptr)
      comp1 = toComparator(class_onto1);
  }
  else if(indiv_onto2 == nullptr)
  {
    comp1 = toComparator(indiv_onto1);
    ClassBranch_t* class_onto2 = onto2->class_graph_.findBranch(concept);
    if(class_onto2 == nullptr)
      comp2 = toComparator(class_onto2);
  }
  else
  {
    comp1 = toComparator(indiv_onto1);
    comp2 = toComparator(indiv_onto2);
  }

  return compare(comp1, comp2);
}

std::vector<std::string> differenceFinder::compare(comparator_t& comp1, comparator_t& comp2)
{
  std::vector<std::string> res;

  return res;
}

comparator_t differenceFinder::toComparator(IndividualBranch_t* indiv)
{
  comparator_t comp;
  comp.concept_ = (ValuedNode*)indiv;
  comp.object_properties_name_ = toValued(indiv->object_properties_name_);
  comp.object_properties_on_ = toValued(indiv->object_properties_on_);
  comp.data_properties_name_ = toValued(indiv->data_properties_name_);
  comp.data_properties_data_ = indiv->data_properties_data_;
  comp.mothers_ = toValued(indiv->is_a_);
  return comp;
}

comparator_t differenceFinder::toComparator(ClassBranch_t* class_)
{
  comparator_t comp;
  comp.concept_ = (ValuedNode*)class_;
  comp.object_properties_name_ = toValued(class_->object_properties_name_);
  comp.object_properties_on_ = toValued(class_->object_properties_on_);
  comp.data_properties_name_ = toValued(class_->data_properties_name_);
  comp.data_properties_data_ = class_->data_properties_data_;
  comp.mothers_ = toValued(class_->mothers_);
  return comp;
}
