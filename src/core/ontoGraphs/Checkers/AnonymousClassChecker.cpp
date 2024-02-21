#include "ontologenius/core/ontoGraphs/Checkers/AnonymousClassChecker.h"

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

size_t AnonymousClassChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(ano_class_graph_->mutex_);
  
  checkDisjoint();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void AnonymousClassChecker::checkDisjoint()
{
  for(AnonymousClassBranches_t* branch_vect : graph_vect_)
  {
    current_ano_ = branch_vect->class_equiv_->value();
    for(auto branch : branch_vect->ano_elems_ )
    {
      auto errs = resolveTree(branch, {ClassElement_t(branch_vect->class_equiv_)});
      for(auto& err : errs)
        print_error("In equivalence of class " + current_ano_ + ": error between class " + current_ano_ + " " + err);
    }
  }
}

std::string AnonymousClassChecker::checkClassesDisjointness(ClassBranch_t* class_left, ClassBranch_t* class_right)
{
  std::string err;
  std::unordered_set<ClassBranch_t*> disjoints;
  
  ano_class_graph_->class_graph_->getDisjoint(class_left, disjoints);

  ClassBranch_t* first_crash = nullptr;
  if(disjoints.size())
  {
    std::unordered_set<ClassBranch_t*> ups;
    ano_class_graph_->class_graph_->getUpPtr(class_right, ups);
    first_crash = ano_class_graph_->class_graph_->firstIntersection(ups, disjoints);
  }

  if(first_crash != nullptr)
  {
    std::unordered_set<ClassBranch_t*> intersection_ups;
    ano_class_graph_->class_graph_->getUpPtr(first_crash, intersection_ups);
    std::unordered_set<ClassBranch_t*> left_ups;
    ano_class_graph_->class_graph_->getUpPtr(class_left, left_ups);

    ClassBranch_t* explanation_1 = nullptr;
    ClassBranch_t* explanation_2 = nullptr;
    for(auto up : intersection_ups)
    {
      explanation_2 = up;
      explanation_1 = ano_class_graph_->class_graph_->firstIntersection(left_ups, up->disjoints_);
      if(explanation_1 != nullptr)
        break;
    }
    
    std::string exp_str = "";
    if (class_right != explanation_2)
      exp_str = class_right->value() + " is a " + explanation_2->value();
    if (class_left != explanation_1)
    {
      if(exp_str != "")
        exp_str += " and ";
      exp_str += class_left->value() + " is a " + explanation_1->value();
    }

    if(explanation_1 == nullptr)
      err = "disjointness between " + class_left->value() + " and " + class_right->value() + 
      " over " + first_crash->value();
    else if (exp_str != "")
      err = "disjointness between " + class_left->value() + " and " + class_right->value() + 
      " because " + explanation_1->value() + " and " + explanation_2->value() + " are disjoint" +
      " and " + exp_str;
    else
      err = "disjointness between " + class_left->value() + " and " + class_right->value() + 
      " because " + explanation_1->value() + " and " + explanation_2->value() + " are disjoint";
  }

  return err;
}

std::vector<std::string> AnonymousClassChecker::checkClassesVectorDisjointness(const std::vector<ClassElement_t>& classes_left, const std::vector<ClassElement_t>& class_right)
{
  std::vector<std::string> errs;
  for(auto& elem_right : class_right)
    for(auto& elem_left : classes_left)
    {
      std::string err = checkClassesDisjointness(elem_left.elem, elem_right.elem);
      if(err != "")
        errs.push_back(err);
    }
  return errs;
}

std::vector<std::string> AnonymousClassChecker::resolveTreeDataTypes(AnonymousClassElement_t* ano_elem)
{
  std::vector<std::string> errs;
  std::unordered_set<std::string> types;

  if(ano_elem->logical_type_ == logical_and)
  {
    // check that there is at each level only the same type of literal type :  (boolean and string) -> error / (boolean and not(string)) -> ok
    for(auto sub_elem : ano_elem->sub_elements_)
    {
      if(sub_elem->logical_type_ == logical_none)
        types.insert(sub_elem->card_.card_range_->type_);
      else
      {
        auto tmp = resolveTreeDataTypes(sub_elem);
        if(tmp.size())
          errs.insert(errs.end(), tmp.begin(), tmp.end());
      }
    }

    if(types.size() > 1)
    {
      std::string exp = "";
      for(auto& type : types)
      {
        if(exp != "")
          exp += ", ";
        exp += type;
      }
      errs.push_back("Data property cannot be of different data types, used types are: " + exp);
    }
  }

  return errs;
}

std::vector<std::string> AnonymousClassChecker::resolveTree(AnonymousClassElement_t* ano_elem, const std::vector<ClassElement_t>& ranges)
{
  std::vector<std::string> errs;
  if(ano_elem->logical_type_ == logical_none && ano_elem->oneof == false)
  {
    auto tmp = checkExpressionDisjointess(ano_elem, ranges);
    if(tmp.size())
      errs.insert(errs.end(), tmp.begin(), tmp.end());
  }
  else if(ano_elem->oneof == true)
  { // check for the OneOf node if the classes of each individuals have no disjunction with the equivalence class
    for(auto elem : ano_elem->sub_elements_)
    {
      auto local_errs = checkClassesVectorDisjointness(ranges, elem->individual_involved_->is_a_.relations);
      if(local_errs.size())
      {
        for(auto& err : local_errs)
          print_error("In equivalence of class " + current_ano_ + ": individual " + elem->individual_involved_->value() +
                      " can not be equivalent to " + current_ano_ + " since it exists a " + err);
      }
    }
  }
  else
  {
    if(ano_elem->logical_type_ == logical_and)
      checkIntersectionDomainsDisjointess(ano_elem);

    for(auto sub_elem : ano_elem->sub_elements_)
    {
      auto tmp = resolveTree(sub_elem, ranges);
      if(tmp.size())
        errs.insert(errs.end(), tmp.begin(), tmp.end());
    }
  }
  return errs;
}

std::vector<std::string> AnonymousClassChecker::checkExpressionDisjointess(AnonymousClassElement_t* ano_elem, const std::vector<ClassElement_t>& ranges)
{
  std::vector<std::string> errs;
  // object or data property restriction
  if(ano_elem->object_property_involved_ != nullptr || ano_elem->data_property_involved_ != nullptr)
    errs = checkPropertyDisjointness(ano_elem, ranges);
  // class only restriction
  else if(ano_elem->class_involved_ != nullptr)
  {
    for(auto& elem : ranges)
    {
      std::string err = checkClassesDisjointness(ano_elem->class_involved_, elem.elem);
      if(err != "")
      {
        err = "and class " + ano_elem->class_involved_->value() + " because of " + err;
        errs.push_back(err);
      }
    }
  }
  return errs;
}

std::vector<std::string> AnonymousClassChecker::checkPropertyDisjointness(AnonymousClassElement_t* ano_elem, const std::vector<ClassElement_t>& ranges)
{
  std::vector<std::string> errs;
  if(ano_elem->object_property_involved_ != nullptr)
  {
    errs = checkPropertyDomainDisjointness(ano_elem->object_property_involved_, ranges);
    std::transform(errs.cbegin(), errs.cend(), errs.begin(), [prop = ano_elem->object_property_involved_->value()](const auto& err) {return "and domain of property " + prop + " because of " + err;});
    checkObjectPropertyRangeDisjointness(ano_elem);
  }
  else
  {
    errs = checkPropertyDomainDisjointness(ano_elem->data_property_involved_, ranges);
    std::transform(errs.cbegin(), errs.cend(), errs.begin(), [prop = ano_elem->data_property_involved_->value()](const auto& err) {return "and domain of property " + prop + " because of " + err;});
    checkDataPropertyRangeDisjointness(ano_elem);
  }
  return errs;
}

std::pair<std::string, bool> getDomainOrigin(AnonymousClassElement_t* ano_elem, size_t index)
{
  size_t cpt = 0;
  for(auto sub_elem : ano_elem->sub_elements_)
  {
    std::string origin;
    bool property = true;
    if(sub_elem->logical_type_ == logical_none)
    {
      if(sub_elem->object_property_involved_ != nullptr)
        origin = sub_elem->object_property_involved_->value();
      else if(sub_elem->data_property_involved_ != nullptr)
        origin = sub_elem->data_property_involved_->value();
      else if(sub_elem->class_involved_ != nullptr)
      {
        origin = sub_elem->class_involved_->value();
        property = false;
      } 
    }

    if(origin != "")
    {
      if(cpt == index)
        return {origin, property};
      cpt++;
    }
  }
  return {"", false};
}

// check for dijsunctions between elements in a AND node (Property -> domains_ , Class -> isA, ) ((obj->domains_) and (B->isA))
void AnonymousClassChecker::checkIntersectionDomainsDisjointess(AnonymousClassElement_t* ano_elem)
{
  std::vector<std::vector<ClassElement_t>> all_domains;
  
  for(auto sub_elem : ano_elem->sub_elements_)
  {
    if(sub_elem->logical_type_ == logical_none)
    {
      if(sub_elem->object_property_involved_ != nullptr)
        all_domains.push_back(sub_elem->object_property_involved_->domains_);
      else if(sub_elem->data_property_involved_ != nullptr)
        all_domains.push_back(sub_elem->data_property_involved_->domains_);
      else if(sub_elem->class_involved_ != nullptr)
        all_domains.push_back({ClassElement_t(sub_elem->class_involved_)});
    }
  }

  for(size_t i = 0; i < all_domains.size(); i++)
    for(size_t j = i+1; j < all_domains.size(); j++)
    {
      auto errs = checkClassesVectorDisjointness(all_domains[i], all_domains[j]);
      if(errs.size())
      {
        auto origin_left = getDomainOrigin(ano_elem, i);
        auto origin_right = getDomainOrigin(ano_elem, j);
        std::string err_left = (origin_left.second ? "domain of property " : "class ") + origin_left.first;
        std::string err_right = (origin_right.second ? "domain of property " : "class ") + origin_right.first;
        for(auto& err : errs)
          print_error("In equivalence of class " + current_ano_ + ": error between " + err_left + " and " + err_right +
                      " because of " + err);
      }
    }
}

void AnonymousClassChecker::checkObjectPropertyRangeDisjointness(AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->is_complex == true)
  {
    auto errs = resolveTree(ano_elem->sub_elements_.front(), ano_elem->object_property_involved_->ranges_);
    for(auto& err : errs)
      print_error("In equivalence of class " + current_ano_ + ": error between range of property " +
                  ano_elem->object_property_involved_->value() + " " + err);
  }
  else if(ano_elem->card_.card_type_ == cardinality_value)
  {
    auto errs = checkClassesVectorDisjointness(ano_elem->object_property_involved_->ranges_, ano_elem->individual_involved_->is_a_.relations);
    for(auto& err : errs)
      print_error("In equivalence of class " + current_ano_ + ": error between range of property " + ano_elem->object_property_involved_->value() +
                  " and inheritence of individual " + ano_elem->individual_involved_->value() + " because of " + err);
  }
  else
  {
    for(auto& range_elem : ano_elem->object_property_involved_->ranges_)
    {
      std::string err = checkClassesDisjointness(range_elem.elem, ano_elem->class_involved_);
      if(err != "")
        print_error("In equivalence of class " + current_ano_ + ": error between range of property " + ano_elem->object_property_involved_->value() +
                    " and class " + ano_elem->class_involved_->value() + " because of " + err);
    }
  }
}

void AnonymousClassChecker::checkDataPropertyRangeDisjointness(AnonymousClassElement_t* ano_elem)
{
  if(ano_elem->is_complex == true)
  {
    auto errs = resolveTreeDataTypes(ano_elem->sub_elements_.front());
    for(auto& err : errs)
      print_error("In equivalence of class " + current_ano_ + ": in ranges of data property " +
                  ano_elem->data_property_involved_->value() + ". " + err);
  }
}



}   // namespace ontologenius