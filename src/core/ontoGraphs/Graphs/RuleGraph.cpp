#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

#include <algorithm>
#include <iostream>

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
      // if(elem_antec.first->is_complex)
      std::vector<std::string> vect_atom = elem_antec.first->rest.toVector();

      if(vect_atom.size() == 1)
      {
        if(!elem_antec.first->rest.restriction_range.empty()) // single class atom
        {
          ClassAtom* class_atom;
          class_atom->class_expression->class_involved_ = class_graph_->findOrCreateBranch(elem_antec.first->rest.restriction_range);
          rule_antecedents->class_atoms_.push_back(class_atom);

          if(elem_antec.second.size() == 1)
          {
            std::cout << elem_antec.second.front() << std::endl;
            if(isIn("indiv#", elem_antec.second.front()))
              class_atom->individual_involved = individual_graph_->findOrCreateBranch(getName(elem_antec.second.front()));
            else if(isIn("var#", elem_antec.second.front()))
              class_atom->var = getName(elem_antec.second.front());
            else
              return nullptr;
          }
        }
        else if(elem_antec.first->is_data_property) // data property atom
        {
          DataPropertyAtom* data_atom;
          data_atom->data_property_expression = data_property_graph_->findOrCreateBranch(elem_antec.first->rest.property);
          rule_antecedents->data_atoms_.push_back(data_atom);

          if(elem_antec.second.size() == 2)
          {
            // get argument 1
            std::string elem1 = elem_antec.second.front();
            if(isIn("indiv#", elem1))
              data_atom->individual_involved = individual_graph_->findOrCreateBranch(getName(elem1));
            else if(isIn("var#", elem1))
              data_atom->var1 = getName(elem1);
            else
              return nullptr;

            // get argument 2
            std::string elem2 = elem_antec.second.back();
            if(isIn("var#", elem2))
              data_atom->var2 = getName(elem2);
            else
            {
              std::string type_datarange = elem2.substr(0, elem2.find("#") - 1);
              data_atom->datatype_involved = data_property_graph_->createLiteral(type_datarange + "#");
            }
          }
        }
        else // object property atom
        {
          ObjectPropertyAtom* object_atom;
          object_atom->object_property_expression = object_property_graph_->findOrCreateBranch(elem_antec.first->rest.property);
          rule_antecedents->object_atoms_.push_back(object_atom);

          if(elem_antec.second.size() == 2)
          {
            // get argument 1
            std::string elem1 = elem_antec.second.front();
            if(isIn("indiv#", elem1))
              object_atom->individual_involved_1 = individual_graph_->findOrCreateBranch(getName(elem1));
            else if(isIn("var#", elem1))
              object_atom->var1 = getName(elem1);
            else
              return nullptr;

            // get argument 2
            std::string elem2 = elem_antec.second.back();
            if(isIn("indiv#", elem2))
              object_atom->individual_involved_2 = individual_graph_->findOrCreateBranch(getName(elem2));
            else if(isIn("var#", elem2))
              object_atom->var2 = getName(elem2);
            else
              return nullptr;
          }
        }
      }
      else // complex class atom
      {
        ClassAtom* class_atom;
        rule_antecedents->class_atoms_.push_back(class_atom);
        // class_atom->class_expression = anonymous_graph_->createTree(ano.equivalence_trees[i], depth);
      }
    }

    // explore consequents
    // for(auto& elem_conseq : rule.consequents)
    // {
    // }

    //     ClassBranch* class_branch = class_graph_->findOrCreateBranch(value);

    //     anonymous_branch->class_equiv_ = class_branch;
    //     all_branchs_.push_back(anonymous_branch);
    //     class_branch->equiv_relations_ = anonymous_branch;

    //     for(size_t i = 0; i < ano.equivalence_trees.size(); i++)
    //     {
    //       size_t depth = 0;
    //       AnonymousClassElement* ano_elem = createTree(ano.equivalence_trees[i], depth);
    //       ano_elem->ano_name = ano_name + "_" + std::to_string(i);
    //       anonymous_branch->ano_elems_.push_back(ano_elem);
    //       anonymous_branch->depth_ = depth;

    // #ifdef DEBUG
    //       printTree(ano_elem, 3, true);
    // #endif
    //     }

    return rule_branch;
  }

  // void RuleGraph::createVariable(std::string variable)
  // {
  //   std::size_t pos = variable.find('#');
  //   if(pos != std::string::npos)
  //   {
  //     std::string type_var = variable.substr(0, pos);
  //     if(type_var == "indiv")
  //       res += variable.substr(pos + 1);
  //     else if(type_var == "var")
  //       res += variable.substr(pos + 1);
  //     else
  //       res += variable.substr(pos + 1);
  //   }
  // }

} // namespace ontologenius