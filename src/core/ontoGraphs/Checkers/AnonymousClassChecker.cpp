#include "ontologenius/core/ontoGraphs/Checkers/AnonymousClassChecker.h"

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

size_t AnonymousClassChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(ano_class_graph_->mutex_);
  graph_size = graph_vect_.size();
  
  checkDisjoint();

  is_analysed = true;
  printStatus();


  return getErrors();
}

void AnonymousClassChecker::checkDisjoint()
{
  for(AnonymousClassBranch_t* branch : graph_vect_)
  {
    std::cout <<" ano branch : " << branch->value() << " with equiv to class " << branch->class_equiv_->value() << std::endl;
    // store the equivalent class disjoint members
    std::unordered_set<ClassBranch_t*> up;
    ano_class_graph_->class_graph_->getUpPtr(branch->class_equiv_, up);
  
    // loop over mothers of the equiv class
    std::unordered_set<ClassBranch_t*> disjoints;
    for(ClassBranch_t* it : up)
    {
      for(auto& disjoint : it->disjoints_)
        disjoints.insert(disjoint.elem);
    }
    std::vector<ClassElement_t> root;
    compareDomainRange(branch->ano_elem_, root);

    // top bottom search -> compare the disjointness of the restriction to the disjoint members of the equiv class
  }
}

void AnonymousClassChecker::compareDomainRange(AnonymousClassElement_t* node, std::vector<ClassElement_t> mother_ranges)
{
  std::cout << "node : ";
  if(node->negation)
  {
    std::cout << "Negation node " << std::endl;
    compareDomainRange(node->sub_elements_.front(), mother_ranges);
  }
  else if(node->andor){
    // take into account ? : (hasComponent some Camera) and (hasCamera some Camera) -> erreur si domaine de hasComponent disjoint avec domaine de hasCamera
    std::cout << "Intersection node" << std::endl;
    for(auto elem : node->sub_elements_)
      compareDomainRange(elem, mother_ranges);
  }
  else if(!node->andor && node->nb_sub > 1)
  {
    std::cout << "Union node" << std::endl;
    for(auto elem : node->sub_elements_)
      compareDomainRange(elem, mother_ranges);
  }
  else{
    if(node->data_property_involved_ != nullptr)
    {
      std::cout << "Data property : " << node->data_property_involved_->value() << " ";
      std::cout << "Domain : ";
      for(auto elem : node->data_property_involved_->domains_)
        std::cout << " " << elem.elem->value() << " ";
      std::cout << " / "; 
      std::cout << "Range : ";
      for(auto elem : node->data_property_involved_->ranges_)
        std::cout << " " << elem->type_ << " ";
      std::cout << std::endl;

      //check whether the domain of the property matches the range of the mother property
      for(auto mother_elem : mother_ranges)
        for(auto current_elem : node->data_property_involved_->domains_)
          checkClassesDisjointness(mother_elem.elem, current_elem.elem);

      if(node->card_.card_range_ != nullptr)
      {
        std::cout << "Simple range Data Property " << node->data_property_involved_->value() << std::endl;
        std::cout << " number : " <<  node->card_.card_number_ <<  "type : " << node->card_.card_type_ << " range " << node->card_.card_range_->type_ << std::endl;
        
        if(node->data_property_involved_->ranges_.size() != 0)
        {
          bool found = false;
          std::string range_dp, range_rest;
          for(auto elem : node->data_property_involved_->ranges_)
          {
            range_dp += elem->type_ + ", ";
            if(node->card_.card_range_->type_ == elem->type_)
              found = true;
          }
          if(!found)
             print_error("The ranges of the DataProperty " + node->data_property_involved_->value() + 
             " (R: " + range_dp  + ") doesn't match with the one in the restriction (R :" + node->card_.card_range_->type_ + ")" );
        }
        
      }
      // complex range
      else
      {  
        std::cout << "Complex range Data Property " << node->data_property_involved_->value() << std::endl;
        // for(auto elem : node->sub_elements_)
        //   if(elem->)
        //compareDomainRange(node->sub_elements_.front(), node->data_property_involved_->ranges_);
        
      }
    }
    else if(node->object_property_involved_ != nullptr)
    {
      std::cout << "Object property : " << node->object_property_involved_->value() << " ";
      std::cout << "Domain : ";
      for(auto elem : node->object_property_involved_->domains_)
        std::cout << " " << elem.elem->value() << " ";
      std::cout << " / "; 
      std::cout << "Range : ";
      for(auto elem : node->object_property_involved_->ranges_)
        std::cout << " " << elem.elem->value() << " ";
      std::cout << std::endl;

      //check whether the domain of the property matches the range of the mother property
      for(auto mother_elem : mother_ranges)
        for(auto current_elem : node->object_property_involved_->domains_)
          checkClassesDisjointness(mother_elem.elem, current_elem.elem);

      //check whether the range of the property matches the class pointed at.
      // simple range
      if(node->class_involved_ != nullptr)
      {
        std::cout << "Simple range object property " << node->object_property_involved_->value() << std::endl;
        for(auto range_elem : node->object_property_involved_->ranges_)
          checkClassesDisjointness(range_elem.elem, node->class_involved_);
      }
      // complex range
      else
      {  
        std::cout << "Complex range object property " << node->object_property_involved_->value() << std::endl;
        for(auto elem : node->sub_elements_)
          compareDomainRange(elem, node->object_property_involved_->ranges_);
      }
    }
    else if(node->class_involved_ != nullptr && node->object_property_involved_ == nullptr)
    {
      std::cout << "Class : " << node->class_involved_->value() << " Disjoint with : ";
      for(auto elem : node->class_involved_->disjoints_)
        std::cout << " " << elem.elem->value() << " ";
      std::cout << std::endl;
      for(auto range_elem: mother_ranges)
        checkClassesDisjointness(range_elem.elem, node->class_involved_);
    }
  }
}

void AnonymousClassChecker::checkClassesDisjointness(ClassBranch_t* class_left, ClassBranch_t* class_right)
{
  std::unordered_set<ClassBranch_t*> disjoints_left, disjoints_right;
  std::unordered_set<ClassBranch_t*> test_left, test_right;
  test_left.insert(class_left);
  test_right.insert(class_right);

  ano_class_graph_->class_graph_->getDisjoint(class_left, disjoints_left);
  ano_class_graph_->class_graph_->getDisjoint(class_right, disjoints_right);

  ClassBranch_t* tmp_intersection2 = findIntersection(test_left, disjoints_right);
  ClassBranch_t* tmp_intersection3 = findIntersection(test_right, disjoints_left);

  if(tmp_intersection2 != nullptr || tmp_intersection3 != nullptr)
    print_error("Disjointness between " + class_left->value() + " and " + class_right->value() + ": Intersection over " + tmp_intersection2->value() + " and " + tmp_intersection3->value());
  else
    std::cout << "No disjointness between classes " << class_left->value()<< " and right " << class_right->value() << std::endl;
}

}   // namespace ontologenius