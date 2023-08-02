#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"

#include <algorithm>
#include <iostream>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

#include "ontologenius/utils/String.h"

namespace ontologenius {


AnonymousClassGraph::AnonymousClassGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, 
DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
  individual_graph_ = individual_graph;
}

AnonymousClassElement_t* AnonymousClassGraph::createElement(ExpressionMember_t* exp_leaf)
{
    std::vector<std::vector<std::string>> vect_equiv2;

    AnonymousClassElement_t* ano_element = new AnonymousClassElement_t();

    if(exp_leaf->class_restriction == "")
    {
      vect_equiv2.push_back(exp_leaf->rest.getRestrictionVector());
    }
    else
    {
      std::vector<std::string> vect;
      vect.push_back(exp_leaf->class_restriction);
      vect_equiv2.push_back(vect);
    }

    std::vector<std::string> vect_equiv = vect_equiv2[0];

    if(vect_equiv.size() == 1)
    {
      std::cout << "Class expression equivalence" << std::endl;

      ClassBranch_t* class_branch = class_graph_->findOrCreateBranch(vect_equiv[0]);
      ano_element->class_involved_= class_branch;
    }
    else if(vect_equiv[1] == "value")
    {
      std::cout << "Value expression equivalence" << std::endl;
      // ObjectProperty
      std::string object_prop_str = vect_equiv[0];
      ObjectPropertyBranch_t* object_branch = object_property_graph_->findOrCreateBranch(object_prop_str);
      ano_element->object_property_involved_= object_branch;

      // Individual
      std::string indiv_str = vect_equiv[2];
      IndividualBranch_t* indiv_branch = individual_graph_->findOrCreateBranch(indiv_str);
      ano_element->individual_involved_= indiv_branch;
      //Cardinality
      ano_element->card_.card_type_ = value_;

    }
    else if(isIn("http://www.w3.org/", vect_equiv.back() ) )
    {
      std::cout << "Data property expression equivalence" << std::endl;
      // Data property
      std::string data_prop_str = vect_equiv[0];
      DataPropertyBranch_t* data_branch = data_property_graph_->findOrCreateBranch(data_prop_str);
      ano_element->data_property_involved_= data_branch;
      // Range
      std::string type = split(vect_equiv.back(), "#").back();
      ano_element->card_.card_range_ = new LiteralNode(type, "");
      //Cardinality
      if(vect_equiv.size() == 4)
        ano_element->card_.card_number_ = stoi(vect_equiv[2]);
      else
        ano_element->card_.card_number_ = 0;

      if(vect_equiv[1]=="some")
        ano_element->card_.card_type_ = some_;
      else if(vect_equiv[1]=="only")
        ano_element->card_.card_type_ = only_;
      else if(vect_equiv[1]=="exactly")
        ano_element->card_.card_type_ = exactly_;
      else if(vect_equiv[1]=="min")
        ano_element->card_.card_type_ = min_;
      else
        ano_element->card_.card_type_ = max_;
    }
    else
    {
      std::cout << "Object property expression equivalence" << std::endl;
      // if last element is a Class then the property is an obj prop, if it is a type (e.g boolean) then it is a data property
      // Object property
      std::string object_prop_str = vect_equiv[0];
      ObjectPropertyBranch_t* object_branch = object_property_graph_->findOrCreateBranch(object_prop_str);
      ano_element->object_property_involved_= object_branch;
      // Range
      ClassBranch_t* class_branch = class_graph_->findOrCreateBranch(vect_equiv.back());
      ano_element->class_involved_= class_branch;
      //Cardinality
      if(vect_equiv.size() == 4)
        ano_element->card_.card_number_ = stoi(vect_equiv[2]);
      else
        ano_element->card_.card_number_ = 0;

      if(vect_equiv[1]=="some")
        ano_element->card_.card_type_ = some_;
      else if(vect_equiv[1]=="only")
        ano_element->card_.card_type_ = only_;
      else if(vect_equiv[1]=="exactly")
        ano_element->card_.card_type_ = exactly_;
      else if(vect_equiv[1]=="min")
        ano_element->card_.card_type_ = min_;
      else
        ano_element->card_.card_type_ = max_;
    }

  return ano_element;
}

void AnonymousClassGraph::update(ExpressionMember_t* exp , AnonymousClassElement_t* ano, bool root)
{
    if(exp->nb_sub != 0)
    {
      //std::cout << "AND OR node : " << exp->andor << std::endl;
      // if root, we do not create a new node, we just fill the values with the current expression member
      if(root)
      {
        ano->nb_sub = exp->nb_sub;
        ano->andor = exp->andor;
        for(auto elem: exp->intersects)
          update(elem, ano, false);
      }

      else
      {
        AnonymousClassElement_t* ano_elem = new AnonymousClassElement_t();
        ano_elem->nb_sub = exp->nb_sub;
        ano_elem->andor = exp->andor;
        ano->sub_elements_.push_back(ano_elem);

        for(auto elem: exp->intersects)
          update(elem, ano_elem, false);
      }
    }
    else
    {
      //creation of filled element
      //std::cout << "leaf node " << exp->str_equivalence << std::endl;
      ano->sub_elements_.push_back(createElement(exp));

    }

}

AnonymousClassBranch_t* AnonymousClassGraph::add(const std::string& value, AnonymousClassVectors_t& ano)
{   
    std::lock_guard<std::shared_timed_mutex> lock(Graph<AnonymousClassBranch_t>::mutex_);

    std::string ano_name = "ano"+ std::to_string(cpt_anonymous_classes_);
    auto anonymous_branch = new AnonymousClassBranch_t(ano_name);
    std::cout << "New anonymous branch id : " << anonymous_branch->value() << " =>" << ano.str_equivalences << std::endl;
    cpt_anonymous_classes_++;

    anonymous_classes_.push_back(anonymous_branch);

    // std::string equiv_expr = ano.str_equivalences;
    // size_t begin_of_pattern = equiv_expr.find("(");
    // std::cout << "begin of pattern : " << begin_of_pattern << std::endl;

    // if(begin_of_pattern == std::string::npos){
    //   std::cout << " no parenthesis, single expression" << std::endl;
    //   // add code to create anonymous branch
    // }
    // else{
    //   std::cout << " parenthesis, complex expression" << std::endl;
    //   std::string res;
    //   std::size_t end_pose = getIn(begin_of_pattern, res, equiv_expr, '(', ')');
    //   std::cout <<" expression is : " << res << std::endl;

    //   size_t begin_of_pattern = res.find("(");
    //   if(begin_of_pattern != std::string::npos)
    //   {
    //     std::size_t last_occ = res.find_last_of("(");
    //     std::size_t first_occ = res.find_first_of(")");

    //     std::cout <<"sub : " << res.substr(last_occ, first_occ) << std::endl;
    //   }
    // }


    ExpressionMember_t* root = ano.equivalence;
    AnonymousClassElement_t* ano_elem = new AnonymousClassElement_t();

     // Class equivalent to 
    ClassBranch_t* class_branch = class_graph_->findOrCreateBranch(value);
    anonymous_branch->class_equiv_= class_branch;

    if(root->mother == nullptr){
      if(root->intersects.size() == 0)
      {
        //std::cout << "single expression " << std::endl;
        anonymous_branch->ano_elem_ = createElement(root);
      }
      else
      {
        //std::cout << "complex expression" << std::endl;
        ExpressionMember_t* leaf = root->intersects[0];
        anonymous_branch->ano_elem_= ano_elem;
        update(leaf, ano_elem, true);
      }
    }
  
    return anonymous_branch;
}

AnonymousClassGraph::AnonymousClassGraph(const AnonymousClassGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph)
{
  class_graph_ = class_graph;
  individual_graph_ = individual_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

} // namespace ontologenius