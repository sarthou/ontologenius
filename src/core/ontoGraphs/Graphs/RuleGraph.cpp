#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/utils/String.h"

// #define DEBUG

namespace ontologenius {

  RuleGraph::RuleGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph,
                       IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph)
  {
    class_graph_ = class_graph;
    object_property_graph_ = object_property_graph;
    data_property_graph_ = data_property_graph;
    individual_graph_ = individual_graph;
    anonymous_graph_ = anonymous_graph;
  }

  RuleGraph::RuleGraph(const RuleGraph& other, ClassGraph* class_graph,
                       ObjectPropertyGraph* object_property_graph,
                       DataPropertyGraph* data_property_graph,
                       IndividualGraph* individual_graph,
                       AnonymousClassGraph* anonymous_graph) : class_graph_(class_graph),
                                                               object_property_graph_(object_property_graph),
                                                               data_property_graph_(data_property_graph),
                                                               individual_graph_(individual_graph),
                                                               anonymous_graph_(anonymous_graph)
  {
    for(auto* branch : other.all_branchs_)
    {
      auto* rule_branch = new RuleBranch(branch->value());
      all_branchs_.push_back(rule_branch);
    }
  }

  RuleBranch* RuleGraph::add(const std::size_t& rule_id, Rule_t& rule)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<RuleBranch>::mutex_);

    const std::string rule_name = "rule_" + std::to_string(rule_id);
    RuleBranch* rule_branch = new RuleBranch(rule_name);
    all_branchs_.push_back(rule_branch);

    size_t elem_id = 0;

    for(auto& elem_antec : rule.antecedents)
    {
      if(elem_antec.first != nullptr)
        createRuleAtomList(&(rule_branch->rule_antecedents_), elem_antec, rule_id, elem_id);
      elem_id++;
    }

    for(auto& elem_conseq : rule.consequents)
    {
      if(elem_conseq.first != nullptr)
        createRuleAtomList(&(rule_branch->rule_consequents_), elem_conseq, rule_id, elem_id);
      elem_id++;
    }

    return rule_branch;
  }

  void RuleGraph::createRuleAtomList(RuleAtomList_t* rule_list, std::pair<ontologenius::ExpressionMember_t*, std::vector<ontologenius::Variable_t>> rule_element, const size_t& rule_id, const size_t& elem_id)
  {
    auto* rule_atom = rule_element.first;
    auto rule_variable = rule_element.second;

    if((rule_atom->is_data_property) & (rule_atom->logical_type_ == logical_none) & (!rule_atom->is_complex))
    {
      // std::cout << "single data atom " << rule_atom->rest.toString() << std::endl;
      // DataPropertyAtom_t* data_atom = createDataPropertyAtom(rule_atom, rule_variable);
      rule_list->data_atoms_.push_back(createDataPropertyAtom(rule_atom, rule_variable));
    }
    else if((rule_atom->logical_type_ == logical_none) & (!rule_atom->is_complex) & (rule_atom->rest.restriction_range.empty()))
    {
      // std::cout << "single object atom " << rule_atom->rest.toString() << std::endl;
      // ObjectPropertyAtom_t* object_atom = createObjectPropertyAtom(rule_atom, rule_variable);
      rule_list->object_atoms_.push_back(createObjectPropertyAtom(rule_atom, rule_variable));
    }
    else
    {
      // std::cout << " class atom  " << rule_atom->rest.toString() << std::endl;
      // ClassAtom_t* class_atom = createClassAtom(rule_atom, rule_variable.front(), rule_id, elem_id);
      rule_list->class_atoms_.push_back(createClassAtom(rule_atom, rule_variable.front(), rule_id, elem_id));
    }
  }

  ClassAtom_t* RuleGraph::createClassAtom(ExpressionMember_t* class_member, const Variable_t& variable, const size_t& rule_id, const size_t& elem_id)
  {
    ClassAtom_t* class_atom = new ClassAtom_t();
    AnonymousClassElement* class_elem = new AnonymousClassElement();

    if(class_member->logical_type_ != logical_none || class_member->oneof == true || class_member->is_complex == true || !class_member->rest.property.empty())
    {
      auto* rule_ano_branch = anonymous_graph_->addHiddenRuleElem(rule_id, elem_id, class_member); // returns the newly created ano branch
      // std::cout << "created classAtom " << rule_ano_branch->value() << std::endl;
      class_atom->equivalent_class = rule_ano_branch->class_equiv_; // get the pointer to the newly created hidden class
      class_elem = rule_ano_branch->ano_elems_.front();             // get the content of the expression for the RuleChecker to process it
    }
    else
    {
      class_atom->equivalent_class = class_graph_->findOrCreateBranch(class_member->rest.restriction_range);
      class_elem->class_involved_ = class_atom->equivalent_class;
    }

    // if(class_member->logical_type_ != logical_none || class_member->oneof == true || class_member->is_complex == true)
    // {
    //   // std::cout << "complex atom " << std::endl;
    //   size_t depth = 0;
    //   class_elem = anonymous_graph_->createTree(class_member, depth, nullptr);

    //   // class_elem->is_hidden = true;
    //   rule_ano_branch->ano_elems_.push_back(class_elem);
    //   anonymous_graph_->all_branchs_.push_back(rule_ano_branch);
    // }
    // else if(!class_member->rest.property.empty())
    // {
    //   // std::cout << "single restriction " << std::endl;
    //   class_elem = anonymous_graph_->createElement(class_member);

    //   rule_ano_branch->ano_elems_.push_back(class_elem);
    //   anonymous_graph_->all_branchs_.push_back(rule_ano_branch);
    // }
    // else
    // {
    //   // std::cout << "simple class atom " << std::endl;
    //   class_elem->class_involved_ = class_graph_->findOrCreateBranch(class_member->rest.restriction_range);
    // }

    if(variable.is_instantiated)
      class_atom->individual_involved = individual_graph_->findOrCreateBranch(variable.var_name);
    else
    {
      variable_names_.insert(variable.var_name);
      class_atom->var = variable.var_name;
    }

    class_atom->class_expression = class_elem;

    return class_atom;
  }

  ObjectPropertyAtom_t* RuleGraph::createObjectPropertyAtom(ExpressionMember_t* property_member, std::vector<Variable_t> variable)
  {
    ObjectPropertyAtom_t* object_atom = new ObjectPropertyAtom_t();
    object_atom->object_property_expression = object_property_graph_->findOrCreateBranch(property_member->rest.property);

    if(variable.size() == 2)
    {
      // get argument 1
      if(variable.front().is_instantiated)
        object_atom->individual_involved_1 = individual_graph_->findOrCreateBranch(variable.front().var_name);
      else
      {
        variable_names_.insert(variable.front().var_name);
        object_atom->var1 = variable.front().var_name;
      }

      // get argument 2
      if(variable.back().is_instantiated)
        object_atom->individual_involved_2 = individual_graph_->findOrCreateBranch(variable.back().var_name);
      else
      {
        variable_names_.insert(variable.back().var_name);
        object_atom->var2 = variable.back().var_name;
      }
    }
    else
      return nullptr;

    return object_atom;
  }

  DataPropertyAtom_t* RuleGraph::createDataPropertyAtom(ExpressionMember_t* property_member, std::vector<Variable_t> variable)
  {
    DataPropertyAtom_t* data_atom = new DataPropertyAtom_t();
    data_atom->data_property_expression = data_property_graph_->findOrCreateBranch(property_member->rest.property);

    if(variable.size() == 2)
    {
      // get argument 1
      if(variable.front().is_instantiated)
        data_atom->individual_involved = individual_graph_->findOrCreateBranch(variable.front().var_name);
      else
      {
        variable_names_.insert(variable.front().var_name);
        data_atom->var1 = variable.front().var_name;
      }

      // get argument 2
      if(variable.back().is_datavalue)
        data_atom->datatype_involved = data_property_graph_->createLiteral(variable.back().var_name);
      else
      {
        variable_names_.insert(variable.back().var_name);
        data_atom->var2 = variable.back().var_name;
      }
    }
    else
      return nullptr;

    return data_atom;
  }

} // namespace ontologenius