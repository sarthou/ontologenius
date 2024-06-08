#include "ontologenius/core/ontoGraphs/Checkers/AnonymousClassChecker.h"

#include <algorithm>
#include <cstddef>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

  size_t AnonymousClassChecker::check()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(ano_class_graph_->mutex_);

    checkDisjoint();

    is_analysed = true;
    printStatus();

    return getErrors();
  }

  void AnonymousClassChecker::checkDisjoint()
  {
    for(AnonymousClassBranch* branch_vect : graph_vect_)
    {
      current_ano_ = branch_vect->class_equiv_->value();
      for(auto* branch : branch_vect->ano_elems_)
      {
        auto errs = resolveTree(branch, {ClassElement(branch_vect->class_equiv_)});
        for(auto& err : errs)
          printError("In equivalence of class " + current_ano_ + ": error between class " + current_ano_ + " " + err);
      }
    }
  }

  std::string AnonymousClassChecker::checkClassesDisjointness(ClassBranch* class_left, ClassBranch* class_right)
  {
    std::string err;
    std::unordered_set<ClassBranch*> disjoints;

    ano_class_graph_->class_graph_->getDisjoint(class_left, disjoints);

    ClassBranch* first_crash = nullptr;
    if(disjoints.empty() == false)
    {
      std::unordered_set<ClassBranch*> ups;
      ano_class_graph_->class_graph_->getUpPtr(class_right, ups);
      first_crash = ano_class_graph_->class_graph_->firstIntersection(ups, disjoints);
    }

    if(first_crash != nullptr)
    {
      std::unordered_set<ClassBranch*> intersection_ups;
      ano_class_graph_->class_graph_->getUpPtr(first_crash, intersection_ups);
      std::unordered_set<ClassBranch*> left_ups;
      ano_class_graph_->class_graph_->getUpPtr(class_left, left_ups);

      ClassBranch* explanation_1 = nullptr;
      ClassBranch* explanation_2 = nullptr;
      for(auto* up : intersection_ups)
      {
        explanation_2 = up;
        explanation_1 = ano_class_graph_->class_graph_->firstIntersection(left_ups, up->disjoints_);
        if(explanation_1 != nullptr)
          break;
      }

      std::string exp_str;
      if(class_right != explanation_2)
        exp_str = class_right->value() + " is a " + explanation_2->value();
      if(class_left != explanation_1)
      {
        if(exp_str.empty() == false)
          exp_str += " and ";
        exp_str += class_left->value() + " is a " + explanation_1->value();
      }

      if(explanation_1 == nullptr)
        err = "disjointness between " + class_left->value() + " and " + class_right->value() +
              " over " + first_crash->value();
      else if(exp_str.empty() == false)
        err = "disjointness between " + class_left->value() + " and " + class_right->value() +
              " because " + explanation_1->value() + " and " + explanation_2->value() + " are disjoint" +
              " and " + exp_str;
      else
        err = "disjointness between " + class_left->value() + " and " + class_right->value() +
              " because " + explanation_1->value() + " and " + explanation_2->value() + " are disjoint";
    }

    return err;
  }

  std::vector<std::string> AnonymousClassChecker::checkClassesVectorDisjointness(const std::vector<ClassElement>& classes_left, const std::vector<ClassElement>& class_right)
  {
    std::vector<std::string> errs;
    for(const auto& elem_right : class_right)
      for(const auto& elem_left : classes_left)
      {
        const std::string err = checkClassesDisjointness(elem_left.elem, elem_right.elem);
        if(err.empty() == false)
          errs.push_back(err);
      }
    return errs;
  }

  std::vector<std::string> AnonymousClassChecker::resolveTreeDataTypes(AnonymousClassElement* ano_elem)
  {
    std::vector<std::string> errs;
    std::unordered_set<std::string> types;

    if(ano_elem->logical_type_ == logical_and)
    {
      // check that there is at each level only the same type of literal type :  (boolean and string) -> error / (boolean and not(string)) -> ok
      for(auto* sub_elem : ano_elem->sub_elements_)
      {
        if(sub_elem->logical_type_ == logical_none)
          types.insert(sub_elem->card_.card_range_->type_);
        else
        {
          auto tmp = resolveTreeDataTypes(sub_elem);
          if(tmp.empty() == false)
            errs.insert(errs.end(), tmp.begin(), tmp.end());
        }
      }

      if(types.size() > 1)
      {
        std::string exp;
        for(const auto& type : types)
        {
          if(exp.empty() == false)
            exp += ", ";
          exp += type;
        }
        errs.push_back("Data property cannot be of different data types, used types are: " + exp);
      }
    }

    return errs;
  }

  std::vector<std::string> AnonymousClassChecker::resolveTree(AnonymousClassElement* ano_elem, const std::vector<ClassElement>& ranges)
  {
    std::vector<std::string> errs;
    if(ano_elem->logical_type_ == logical_none && ano_elem->oneof == false)
    {
      auto tmp = checkExpressionDisjointess(ano_elem, ranges);
      if(tmp.empty() == false)
        errs.insert(errs.end(), tmp.begin(), tmp.end());
    }
    else if(ano_elem->oneof == true)
    { // check for the OneOf node if the classes of each individuals have no disjunction with the equivalence class
      for(auto* elem : ano_elem->sub_elements_)
      {
        auto local_errs = checkClassesVectorDisjointness(ranges, elem->individual_involved_->is_a_.relations);
        if(local_errs.empty() == false)
        {
          for(auto& err : local_errs)
            printError("In equivalence of class " + current_ano_ + ": individual " + elem->individual_involved_->value() +
                       " can not be equivalent to " + current_ano_ + " since it exists a " + err);
        }
      }
    }
    else
    {
      if(ano_elem->logical_type_ == logical_and)
        checkIntersectionDomainsDisjointess(ano_elem);

      for(auto* sub_elem : ano_elem->sub_elements_)
      {
        auto tmp = resolveTree(sub_elem, ranges);
        if(tmp.empty() == false)
          errs.insert(errs.end(), tmp.begin(), tmp.end());
      }
    }
    return errs;
  }

  std::vector<std::string> AnonymousClassChecker::checkExpressionDisjointess(AnonymousClassElement* ano_elem, const std::vector<ClassElement>& ranges)
  {
    std::vector<std::string> errs;
    // object or data property restriction
    if(ano_elem->object_property_involved_ != nullptr || ano_elem->data_property_involved_ != nullptr)
      errs = checkPropertyDisjointness(ano_elem, ranges);
    // class only restriction
    else if(ano_elem->class_involved_ != nullptr)
    {
      for(const auto& elem : ranges)
      {
        const std::string err = checkClassesDisjointness(ano_elem->class_involved_, elem.elem);
        if(err.empty() == false)
          errs.emplace_back("and class " + ano_elem->class_involved_->value() + " because of " + err);
      }
    }
    return errs;
  }

  std::vector<std::string> AnonymousClassChecker::checkPropertyDisjointness(AnonymousClassElement* ano_elem, const std::vector<ClassElement>& ranges)
  {
    std::vector<std::string> errs;
    if(ano_elem->object_property_involved_ != nullptr)
    {
      errs = checkPropertyDomainDisjointness(ano_elem->object_property_involved_, ranges);
      std::transform(errs.cbegin(), errs.cend(), errs.begin(), [prop = ano_elem->object_property_involved_->value()](const auto& err) { return "and domain of property " + prop + " because of " + err; });
      checkObjectPropertyRangeDisjointness(ano_elem);
    }
    else
    {
      errs = checkPropertyDomainDisjointness(ano_elem->data_property_involved_, ranges);
      std::transform(errs.cbegin(), errs.cend(), errs.begin(), [prop = ano_elem->data_property_involved_->value()](const auto& err) { return "and domain of property " + prop + " because of " + err; });
      checkDataPropertyRangeDisjointness(ano_elem);
    }
    return errs;
  }

  std::pair<std::string, bool> getDomainOrigin(AnonymousClassElement* ano_elem, size_t index)
  {
    size_t cpt = 0;
    for(auto* sub_elem : ano_elem->sub_elements_)
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

      if(origin.empty() == false)
      {
        if(cpt == index)
          return {origin, property};
        cpt++;
      }
    }
    return {"", false};
  }

  // check for dijsunctions between elements in a AND node (Property -> domains_ , Class -> isA, ) ((obj->domains_) and (B->isA))
  void AnonymousClassChecker::checkIntersectionDomainsDisjointess(AnonymousClassElement* ano_elem)
  {
    std::vector<std::vector<ClassElement>> all_domains;

    for(auto* sub_elem : ano_elem->sub_elements_)
    {
      if(sub_elem->logical_type_ == logical_none)
      {
        if(sub_elem->object_property_involved_ != nullptr)
          all_domains.push_back(sub_elem->object_property_involved_->domains_);
        else if(sub_elem->data_property_involved_ != nullptr)
          all_domains.push_back(sub_elem->data_property_involved_->domains_);
        else if(sub_elem->class_involved_ != nullptr)
          all_domains.push_back({ClassElement(sub_elem->class_involved_)});
      }
    }

    for(size_t i = 0; i < all_domains.size(); i++)
      for(size_t j = i + 1; j < all_domains.size(); j++)
      {
        auto errs = checkClassesVectorDisjointness(all_domains[i], all_domains[j]);
        if(errs.empty() == false)
        {
          auto origin_left = getDomainOrigin(ano_elem, i);
          auto origin_right = getDomainOrigin(ano_elem, j);
          const std::string err_left = (origin_left.second ? "domain of property " : "class ") + origin_left.first;
          const std::string err_right = (origin_right.second ? "domain of property " : "class ") + origin_right.first;
          const std::string err_base = "In equivalence of class " + current_ano_ + ": error between " +
                                       err_left + " and " + err_right + " because of ";
          for(const auto& err : errs)
            printError(err_base + err);
        }
      }
  }

  void AnonymousClassChecker::checkObjectPropertyRangeDisjointness(AnonymousClassElement* ano_elem)
  {
    if(ano_elem->is_complex == true)
    {
      auto errs = resolveTree(ano_elem->sub_elements_.front(), ano_elem->object_property_involved_->ranges_);
      for(const auto& err : errs)
        printError("In equivalence of class " + current_ano_ + ": error between range of property " +
                   ano_elem->object_property_involved_->value() + " " + err);
    }
    else if(ano_elem->card_.card_type_ == cardinality_value)
    {
      auto errs = checkClassesVectorDisjointness(ano_elem->object_property_involved_->ranges_, ano_elem->individual_involved_->is_a_.relations);
      for(const auto& err : errs)
        printError("In equivalence of class " + current_ano_ + ": error between range of property " + ano_elem->object_property_involved_->value() +
                   " and inheritence of individual " + ano_elem->individual_involved_->value() + " because of " + err);
    }
    else
    {
      for(auto& range_elem : ano_elem->object_property_involved_->ranges_)
      {
        const std::string err = checkClassesDisjointness(range_elem.elem, ano_elem->class_involved_);
        if(err.empty() == false)
          printError("In equivalence of class " + current_ano_ + ": error between range of property " + ano_elem->object_property_involved_->value() +
                     " and class " + ano_elem->class_involved_->value() + " because of " + err);
      }
    }
  }

  void AnonymousClassChecker::checkDataPropertyRangeDisjointness(AnonymousClassElement* ano_elem)
  {
    if(ano_elem->is_complex == true)
    {
      auto errs = resolveTreeDataTypes(ano_elem->sub_elements_.front());
      for(auto& err : errs)
        printError("In equivalence of class " + current_ano_ + ": in ranges of data property " +
                   ano_elem->data_property_involved_->value() + ". " + err);
    }
  }

} // namespace ontologenius