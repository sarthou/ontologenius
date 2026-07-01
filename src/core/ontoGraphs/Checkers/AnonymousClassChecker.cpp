#include "ontologenius/core/ontoGraphs/Checkers/AnonymousClassChecker.h"

#include <cstddef>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

  size_t AnonymousClassChecker::check()
  {
    std::shared_lock<std::shared_timed_mutex> lock(graphs_->anonymous_classes_.mutex_);

    checkDisjoint();

    is_analysed = true;
    printStatus();

    return getErrors();
  }

  void AnonymousClassChecker::checkDisjoint()
  {
    for(const AnonymousClassBranch* branch_vect : graph_vect_)
    {
      std::unordered_set<ClassBranch*> disjoints;
      std::unordered_set<ClassBranch*> uppers;

      graphs_->classes_.getDisjoint(branch_vect->class_equiv_, disjoints);
      graphs_->classes_.getUpPtr(branch_vect->class_equiv_, uppers);

      current_ano_ = branch_vect->class_equiv_->value();
      for(auto* tree : branch_vect->ano_trees_)
      {
        auto errs = resolveTreeDisjoint(tree->root_node_, disjoints, uppers, false);

        for(auto& err : errs)
          printError("In equivalence of class " + current_ano_ + ": error " + err);
      }
    }
  }

  std::vector<std::string> AnonymousClassChecker::resolveTreeDisjoint(ClassExpression* ano_elem, std::unordered_set<ClassBranch*>& disjoints, std::unordered_set<ClassBranch*>& uppers, bool complement_mode)
  {
    std::vector<std::string> errs;
    std::string err_inter = "in intersection expression";
    std::string err_union = "in union expression";
    std::string err_compl = "in complement";

    if(ano_elem->type_ == class_expression_identifier)
      return checkIdentifier(ano_elem, disjoints, uppers, complement_mode);
    else if(ano_elem->type_ == class_expression_one_of)
      return checkOneOf(ano_elem, disjoints, uppers, complement_mode);
    else if(ano_elem->type_ == class_expression_restriction)
      return checkRestriction(ano_elem, disjoints, uppers, complement_mode);
    else if(ano_elem->type_ == class_expression_union_of)
    {
      for(auto* sub_elem : ano_elem->sub_elements_)
      {
        std::unordered_set<ClassBranch*> copy_all_disjoint = disjoints;
        auto tmp = resolveTreeDisjoint(sub_elem, copy_all_disjoint, uppers, complement_mode);
        for(auto& err : tmp)
          errs.emplace_back(err_union + ", " + err);
      }
    }
    else if(ano_elem->type_ == class_expression_intersection_of)
    {
      // copy to ensure same parent context
      std::unordered_set<ClassBranch*> new_uppers = uppers;
      for(auto* sub_elem : ano_elem->sub_elements_)
      {
        auto tmp = resolveTreeDisjoint(sub_elem, disjoints, new_uppers, complement_mode);
        for(auto& err : tmp)
          errs.emplace_back(err_inter + ", " + err);
      }
    }
    else if(ano_elem->type_ == class_expression_complement_of)
    {
      // if we were already in complement mode, we switch back to false, otherwise we switch to true
      bool new_mode = !complement_mode;

      std::unordered_set<ClassBranch*> empty_disjoints;
      auto tmp = resolveTreeDisjoint(ano_elem->sub_elements_.front(), empty_disjoints, uppers, new_mode);
      for(auto& err : tmp)
        errs.emplace_back(err_compl + ", " + err);
    }
    return errs;
  }

  std::vector<std::string> AnonymousClassChecker::resolveTreeDisjoint(ClassExpression* ano_elem, std::unordered_set<LiteralType*>& data_ranges)
  {
    std::vector<std::string> errs;
    std::string err_inter = "in intersection expression";
    std::string err_union = "in union expression";
    std::string err_compl = "in complement";

    if(ano_elem->type_ == class_expression_identifier)
      return checkIdentifier(ano_elem, data_ranges);
    else if(ano_elem->type_ == class_expression_one_of)
      return checkOneOf(ano_elem, data_ranges);
    else if(ano_elem->type_ == class_expression_union_of)
    {
      for(auto* sub_elem : ano_elem->sub_elements_)
      {
        std::unordered_set<LiteralType*> copy_all_data_ranges = data_ranges;
        auto tmp = resolveTreeDisjoint(sub_elem, copy_all_data_ranges);
        for(auto& err : tmp)
          errs.emplace_back(err_union + ", " + err);
      }
    }
    else if(ano_elem->type_ == class_expression_intersection_of)
    {
      for(auto* sub_elem : ano_elem->sub_elements_)
      {
        auto tmp = resolveTreeDisjoint(sub_elem, data_ranges);
        for(auto& err : tmp)
          errs.emplace_back(err_inter + ", " + err);
      }
    }
    else if(ano_elem->type_ == class_expression_complement_of)
    {
      std::unordered_set<LiteralType*> empty_data_range;
      // todo: boolean and not(boolean) case with the uppers and complement mode
      auto tmp = resolveTreeDisjoint(ano_elem->sub_elements_.front(), empty_data_range);
      for(auto& err : tmp)
        errs.emplace_back(err_compl + ", " + err);
    }

    return errs;
  }

  std::vector<std::string> AnonymousClassChecker::checkIdentifier(ClassExpression* ano_elem, std::unordered_set<ClassBranch*>& disjoints, std::unordered_set<ClassBranch*>& uppers, bool complement_mode)
  {
    std::vector<std::string> errors;

    if(ano_elem->individual_involved_ != nullptr)
    {
      if(disjoints.empty() == false)
      {
        if(ano_elem->individual_involved_->same_as_.empty() == true)
        {
          const ClassBranch* conflict = graphs_->classes_.firstIntersection(disjoints, ano_elem->individual_involved_->is_a_.relations);
          if(conflict != nullptr)
            errors.emplace_back("individual " + ano_elem->individual_involved_->value() + " has type " + conflict->value() + " which is disjoint to a previous element");
        }
        else
        {
          for(auto& same : ano_elem->individual_involved_->same_as_)
          {
            const ClassBranch* conflict = graphs_->classes_.firstIntersection(disjoints, same.elem->is_a_.relations);

            if(conflict != nullptr)
            {
              if(same.elem != ano_elem->individual_involved_)
                errors.emplace_back("individual " + ano_elem->individual_involved_->value() + " is same as " + same.elem->value() + ", which has type " + conflict->value() + ", which is disjoint to a previous element");
              else
                errors.emplace_back("individual " + ano_elem->individual_involved_->value() + " has type " + conflict->value() + ", which is disjoint to a previous element");
            }
          }
        }
      }

      // check that the class of the individual is not a part of the uppers (only in the complement mode)
      if(complement_mode == true && uppers.empty() == false)
      {
        const ClassBranch* conflict = graphs_->classes_.firstIntersection(uppers, ano_elem->individual_involved_->is_a_.relations);
        if(conflict != nullptr)
          errors.emplace_back("unsatisfiable expression with individual " + ano_elem->individual_involved_->value() + " which has type " + conflict->value() + ", which is in the upper classes");
      }

      for(auto& class_elem : ano_elem->individual_involved_->is_a_)
        graphs_->classes_.getDisjoint(class_elem.elem, disjoints);

      for(auto& class_elem : ano_elem->individual_involved_->is_a_)
        graphs_->classes_.getUpPtr(class_elem.elem, uppers);
    }
    else if(ano_elem->class_involved_ != nullptr)
    {
      if(disjoints.empty() == false && disjoints.find(ano_elem->class_involved_) != disjoints.end())
        errors.emplace_back("class " + ano_elem->class_involved_->value() + " is disjoint to a previous element ");

      // check that the class is not a part of the uppers (only in the complement mode)
      if(complement_mode == true && uppers.empty() == false && uppers.find(ano_elem->class_involved_) != uppers.end())
        errors.emplace_back("unsatisfiable expression with class " + ano_elem->class_involved_->value() + " which is in the upper classes");

      graphs_->classes_.getDisjoint(ano_elem->class_involved_, disjoints);
      graphs_->classes_.getUpPtr(ano_elem->class_involved_, uppers);
    }
    else
      printError("In equivalence of class " + current_ano_ + ": Identifier involves non-individual or class element.");

    return errors;
  }

  std::vector<std::string> AnonymousClassChecker::checkIdentifier(ClassExpression* ano_elem, std::unordered_set<LiteralType*>& data_ranges)
  {
    std::vector<std::string> errors;
    if(data_ranges.empty())
      return errors;

    // required to check whether the ranges types of a data property are compatible (ex : string and boolean -> false)
    if(ano_elem->literal_involved_ != nullptr) // integer#12
    {
      if(data_ranges.find(ano_elem->literal_involved_->type_) == data_ranges.end())
        errors.emplace_back("literal " + ano_elem->literal_involved_->value() + " has type " + ano_elem->literal_involved_->type_->value() + "  is different to an upper type");
    }
    else if(ano_elem->datatype_involved_ != nullptr) // boolean
    {
      if(data_ranges.find(ano_elem->datatype_involved_) == data_ranges.end())
        errors.emplace_back("datatype " + ano_elem->datatype_involved_->value() + " is different to an upper type");
    }
    else
      printError("In equivalence of class " + current_ano_ + ": Identifier involves non-literal or datatype element.");

    return errors;
  }

  std::vector<std::string> AnonymousClassChecker::checkOneOf(ClassExpression* ano_elem, std::unordered_set<ClassBranch*>& disjoints, std::unordered_set<ClassBranch*>& uppers, bool complement_mode)
  {
    std::vector<std::string> errors;

    for(auto* elem : ano_elem->sub_elements_)
    {
      if(elem->individual_involved_ == nullptr)
        printError("In equivalence of class " + current_ano_ + ": oneOf involves non-individual element.");
      else
      {
        std::unordered_set<ClassBranch*> copy_disjoints = disjoints;
        auto tmp = checkIdentifier(elem, copy_disjoints, uppers, complement_mode);
        for(auto& err : tmp)
          errors.emplace_back("in oneOf, " + err);
      }
    }

    return errors;
  }

  std::vector<std::string> AnonymousClassChecker::checkOneOf(ClassExpression* ano_elem, std::unordered_set<LiteralType*>& data_ranges)
  {
    std::vector<std::string> errors;
    if(data_ranges.empty())
      return errors;

    for(auto* elem : ano_elem->sub_elements_)
    {
      if(elem->literal_involved_ == nullptr)
        printError("In equivalence of class " + current_ano_ + ": oneOf involves non-literal element.");
      else
      {
        if(data_ranges.find(ano_elem->literal_involved_->type_) == data_ranges.end())
          errors.emplace_back("in oneOf: literal " + elem->literal_involved_->value() + " has type " + elem->literal_involved_->type_->value() + " which is different to an upper type");
        data_ranges.insert(ano_elem->literal_involved_->type_);
      }
    }

    return errors;
  }

  std::vector<std::string> AnonymousClassChecker::checkRestriction(ClassExpression* ano_elem, std::unordered_set<ClassBranch*>& disjoints, std::unordered_set<ClassBranch*>& uppers, bool complement_mode)
  {
    std::vector<std::string> errors;
    std::string err_restriction = "in restriction,";
    // Domain check

    if(ano_elem->object_property_involved_ != nullptr)
    {
      const ClassBranch* conflict = graphs_->classes_.firstIntersection(disjoints, ano_elem->object_property_involved_->domains_);
      if(conflict != nullptr)
        errors.emplace_back(err_restriction + " property " + ano_elem->object_property_involved_->value() + " has domain " + conflict->value() + " which is disjoint to a previous element");

      for(auto& class_elem : ano_elem->object_property_involved_->domains_)
        graphs_->classes_.getDisjoint(class_elem.elem, disjoints);
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      const ClassBranch* conflict = graphs_->classes_.firstIntersection(disjoints, ano_elem->data_property_involved_->domains_);
      if(conflict != nullptr)
        errors.emplace_back(err_restriction + " property " + ano_elem->data_property_involved_->value() + " has domain " + conflict->value() + " which is disjoint to a previous element");

      for(auto& class_elem : ano_elem->data_property_involved_->domains_)
        graphs_->classes_.getDisjoint(class_elem.elem, disjoints);
    }

    // Complement check
    if(complement_mode == true)
    {
      if(ano_elem->object_property_involved_ != nullptr)
      {
        if(uppers.empty() == false)
        {
          const ClassBranch* conflict = graphs_->classes_.firstIntersection(uppers, ano_elem->object_property_involved_->domains_);
          if(conflict != nullptr)
            errors.emplace_back(err_restriction + "unsatisfiable expression with property " + ano_elem->object_property_involved_->value() + " that has domain " + conflict->value() + ", which is in the negative classes");
        }

        for(auto& class_elem : ano_elem->object_property_involved_->domains_)
          graphs_->classes_.getUpPtr(class_elem.elem, uppers);
      }
      else if(ano_elem->data_property_involved_ != nullptr)
      {
        if(uppers.empty() == false)
        {
          const ClassBranch* conflict = graphs_->classes_.firstIntersection(uppers, ano_elem->data_property_involved_->domains_);
          if(conflict != nullptr)
            errors.emplace_back(err_restriction + "unsatisfiable expression with property " + ano_elem->data_property_involved_->value() + " that has domain " + conflict->value() + ", which is in the negative classes");
        }

        for(auto& class_elem : ano_elem->data_property_involved_->domains_)
          graphs_->classes_.getUpPtr(class_elem.elem, uppers);
      }
    }

    // Range check
    if(ano_elem->object_property_involved_ != nullptr)
    {
      std::unordered_set<ClassBranch*> disjoints_ranges;
      std::unordered_set<ClassBranch*> new_uppers;

      // updates the sets of disjointness and uppers with current values
      for(auto& range : ano_elem->object_property_involved_->ranges_)
      {
        graphs_->classes_.getDisjoint(range.elem, disjoints_ranges);
        graphs_->classes_.getUpPtr(range.elem, new_uppers);
      }

      for(auto* elem : ano_elem->sub_elements_)
      {
        auto tmp = resolveTreeDisjoint(elem, disjoints_ranges, new_uppers, false);
        for(auto& err : tmp)
          errors.emplace_back(err_restriction + " range of property " + ano_elem->object_property_involved_->value() + ", " + err);
      }
    }
    else if(ano_elem->data_property_involved_ != nullptr)
    {
      std::unordered_set<LiteralType*> data_ranges;
      for(auto* range : ano_elem->data_property_involved_->ranges_)
        data_ranges.insert(range);

      for(auto* elem : ano_elem->sub_elements_)
      {
        auto tmp = resolveTreeDisjoint(elem, data_ranges);
        for(auto& err : tmp)
          errors.emplace_back(err_restriction + " range of property " + ano_elem->data_property_involved_->value() + ", " + err);
      }
    }
    else
      printError("In equivalence of class " + current_ano_ + ": restriction does not involve property.");

    return errors;
  }

  std::vector<std::string> AnonymousClassChecker::resolveTreeDataTypes(ClassExpression* ano_elem, std::unordered_set<LiteralType*>& data_ranges)
  {
    std::vector<std::string> errs;
    std::unordered_set<std::string> types;

    if(ano_elem->type_ == class_expression_intersection_of)
    {
      // check that there is at each level only the same type of literal type :  (boolean and string) -> error / (boolean and not(string)) -> ok
      for(auto* sub_elem : ano_elem->sub_elements_)
      {
        if(sub_elem->type_ == class_expression_identifier)
          types.insert(sub_elem->datatype_involved_->value());
        else
        {
          auto tmp = resolveTreeDataTypes(sub_elem, data_ranges);
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
        printError("Data property cannot be of different data types, used types are: " + exp);
      }
    }
    else if(ano_elem->type_ == class_expression_union_of)
    {
      for(auto* sub_elem : ano_elem->sub_elements_)
      {
        if(sub_elem->type_ == class_expression_identifier)
          types.insert(sub_elem->datatype_involved_->value());
        else
        {
          auto tmp = resolveTreeDataTypes(sub_elem, data_ranges);
          if(tmp.empty() == false)
            errs.insert(errs.end(), tmp.begin(), tmp.end());
        }
      }
    }
    else if(ano_elem->type_ == class_expression_identifier)
    {
      auto tmp = checkIdentifier(ano_elem, data_ranges);
      errs.insert(errs.end(), tmp.begin(), tmp.end());
    }
    else if(ano_elem->type_ == class_expression_complement_of)
    {
      std::unordered_set<LiteralType*> new_data_ranges;
      auto tmp = resolveTreeDataTypes(ano_elem->sub_elements_.front(), new_data_ranges);
      errs.insert(errs.end(), tmp.begin(), tmp.end());
    }

    return errs;
  }

} // namespace ontologenius