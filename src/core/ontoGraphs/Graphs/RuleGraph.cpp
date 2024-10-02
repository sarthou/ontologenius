#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

#include <algorithm>
#include <iostream>

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

  RuleBranch* RuleGraph::add(const std::size_t& value, Rule_t& rule)
  {
    const std::lock_guard<std::shared_timed_mutex> lock(Graph<RuleBranch>::mutex_);

    const std::string rule_name = "rule_" + std::to_string(value);
    RuleBranch* rule_branch = new RuleBranch(rule_name);
    all_branchs_.push_back(rule_branch);
    // explore antecedents
    RuleAtomList* rule_antecedents = new RuleAtomList();
    rule_branch->rule_antecedents_ = rule_antecedents;

    RuleAtomList* rule_consequents = new RuleAtomList();
    rule_branch->rule_consequents_ = rule_consequents;

    for(auto& elem_antec : rule.antecedents)
    { // if atom is a class expression, create an unamed anonymous class
      if(elem_antec.first != nullptr)
      {
        if(elem_antec.first->logical_type_ != logical_none || elem_antec.first->oneof == true || elem_antec.first->is_complex == true)
        {
          std::cout << "complex atom " << std::endl;
          ClassAtom* class_atom = new ClassAtom();
          rule_antecedents->class_atoms_.push_back(class_atom);
          size_t depth = 0;

          AnonymousClassElement* class_elem = anonymous_graph_->createTree(elem_antec.first, depth);
          class_atom->class_expression = class_elem;
        }
        else if(!elem_antec.first->rest.property.empty() && (!elem_antec.first->rest.restriction_range.empty() || !elem_antec.first->rest.card.cardinality_range.empty()))
        {
          std::cout << "single restriction " << std::endl;
          ClassAtom* class_atom = new ClassAtom();
          rule_antecedents->class_atoms_.push_back(class_atom);

          AnonymousClassElement* class_elem = anonymous_graph_->createElement(elem_antec.first);
          class_atom->class_expression = class_elem;
        }
        else // if the atom is of size 1, then it is either a class only atom, object or data property atom
        {
          std::cout << "single atom " << std::endl;
          if(!elem_antec.first->rest.restriction_range.empty()) // single class atom
          {
            std::cout << "single class atom with rest : " << elem_antec.first->rest.toString() << std::endl;
            ClassAtom* class_atom = new ClassAtom();
            AnonymousClassElement* class_elem = new AnonymousClassElement();
            class_elem->class_involved_ = class_graph_->findOrCreateBranch(elem_antec.first->rest.restriction_range);
            class_atom->class_expression = class_elem;
            rule_antecedents->class_atoms_.push_back(class_atom);

            std::vector<Variable_t> current_var = elem_antec.second;

            if(elem_antec.second.size() == 1)
            {
              Variable_t current_var;
              if(current_var.is_instantiated)
                class_atom->individual_involved = individual_graph_->findOrCreateBranch(current_var.var_name);
              else
                class_atom->var = current_var.var_name;
            }
          }
          else if(elem_antec.first->is_data_property) // data property atom
          {
            std::cout << "single data atom " << elem_antec.first->rest.toString() << std::endl;
            DataPropertyAtom* data_atom = new DataPropertyAtom();
            data_atom->data_property_expression = data_property_graph_->findOrCreateBranch(elem_antec.first->rest.property);
            rule_antecedents->data_atoms_.push_back(data_atom);

            if(elem_antec.second.size() == 2)
            {
              // get argument 1
              Variable_t var1 = elem_antec.second.front();
              if(var1.is_instantiated)
                data_atom->individual_involved = individual_graph_->findOrCreateBranch(var1.var_name);
              else
                data_atom->var1 = var1.var_name;

              // get argument 2
              Variable_t var2 = elem_antec.second.back();
              if(var2.is_datavalue)
              {
                std::string type_datarange = var2.var_name.substr(0, var2.var_name.find("#") - 1);
                data_atom->datatype_involved = data_property_graph_->createLiteral(type_datarange + "#");
              }
              else
                data_atom->var2 = var2.var_name;
            }
          }
          else // object property atom
          {
            std::cout << "single object atom " << elem_antec.first->rest.toString() << std::endl;
            ObjectPropertyAtom* object_atom = new ObjectPropertyAtom();
            object_atom->object_property_expression = object_property_graph_->findOrCreateBranch(elem_antec.first->rest.property);
            rule_antecedents->object_atoms_.push_back(object_atom);

            if(elem_antec.second.size() == 2)
            {
              // get argument 1
              Variable_t var1 = elem_antec.second.front();
              if(var1.is_instantiated)
                object_atom->individual_involved_1 = individual_graph_->findOrCreateBranch(var1.var_name);
              else
                object_atom->var1 = var1.var_name;

              // get argument 2
              Variable_t var2 = elem_antec.second.back();
              if(var2.is_instantiated)
                object_atom->individual_involved_2 = individual_graph_->findOrCreateBranch(var2.var_name);
              else
                object_atom->var2 = var2.var_name;
            }
          }
        }
      }
    }

    // explore consequents
    // for(auto& elem_conseq : rule.consequents)
    // {
    // }
    return rule_branch;
  }

} // namespace ontologenius