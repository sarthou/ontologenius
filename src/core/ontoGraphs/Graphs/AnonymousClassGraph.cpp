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
  cpt_anonymous_classes_ = 0;
}

AnonymousClassGraph::AnonymousClassGraph(const AnonymousClassGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph)
{
  class_graph_ = class_graph;
  individual_graph_ = individual_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

AnonymousClassElement_t* AnonymousClassGraph::createElement(ExpressionMember_t* exp_leaf)
{
  AnonymousClassElement_t* ano_element = new AnonymousClassElement_t();
  std::vector<std::string> vect_equiv = exp_leaf->rest.getRestrictionVector();
  ano_element->nb_sub = exp_leaf->nb_sub;

  if(exp_leaf->logical_type_ != logical_none)
  {
    ano_element->logical_type_ = exp_leaf->logical_type_;
    return ano_element;
  }
  else if(exp_leaf->oneof == true)
  {
    ano_element->oneof = true;
    return ano_element;
  }
  // class, indiv or literal node only
  if(vect_equiv.size() == 1)
  {
    std::string s = vect_equiv.front();
    if(isIn("http://www.w3.org/", s))
    {
      std::string type = split(s, "#").back();
      ano_element->card_.card_range_ = new LiteralNode(type, "");
    }
    else
    {
      if(exp_leaf->mother != nullptr && exp_leaf->mother->oneof)
        ano_element->individual_involved_= individual_graph_->findOrCreateBranch(s);
      else
        ano_element->class_involved_= class_graph_->findOrCreateBranch(s);
    }    
  }
  else if(vect_equiv[1] == "value")
  { 
    ano_element->object_property_involved_= object_property_graph_->findOrCreateBranch(vect_equiv.front());
    ano_element->card_.card_type_ = value_;
    ano_element->individual_involved_= individual_graph_->findOrCreateBranch(vect_equiv[2]);
  }
  else{

      std::string property = vect_equiv.front();

      if(vect_equiv[1]=="some")
        ano_element->card_.card_type_ = some_;
      else if(vect_equiv[1]=="only")
        ano_element->card_.card_type_ = only_;
      else if(vect_equiv[1]=="exactly")
        ano_element->card_.card_type_ = exactly_;
      else if(vect_equiv[1]=="min")
        ano_element->card_.card_type_ = min_;
      else if(vect_equiv[1]=="max")
        ano_element->card_.card_type_ = max_;
      else
        ano_element->card_.card_type_ = error_;

      if(exp_leaf->nb_sub == 1)
      {
        if(exp_leaf->isDataProp)
          ano_element->data_property_involved_= data_property_graph_->findOrCreateBranch(property);
        else
          ano_element->object_property_involved_= object_property_graph_->findOrCreateBranch(property);
        
        if(vect_equiv.size() == 3)
            ano_element->card_.card_number_ = std::stoi(vect_equiv.back());
      }
      else
      {
        if(vect_equiv.size() == 4)
          ano_element->card_.card_number_ = stoi(vect_equiv[2]);
        else
          ano_element->card_.card_number_ = 0;

        // if last element in vect_equiv contains http://www.w3.org/ => data property
        if(isIn("http://www.w3.org/", vect_equiv.back()))
        {
          ano_element->data_property_involved_= data_property_graph_->findOrCreateBranch(property);
          std::string type = split(vect_equiv.back(), "#").back();
          ano_element->card_.card_range_ = new LiteralNode(type, "");
        }
        else
        {
          ano_element->object_property_involved_= object_property_graph_->findOrCreateBranch(property);
          ano_element->class_involved_= class_graph_->findOrCreateBranch(vect_equiv.back());
        }
      }  
    }

  return ano_element;
}

void AnonymousClassGraph::update(ExpressionMember_t* exp , AnonymousClassElement_t* ano, bool root)
{
  if(exp->logical_type_ != logical_none || exp->oneof == true || exp->nb_sub == 1)
  {
    AnonymousClassElement_t* ano_tmp = createElement(exp);
    ano->sub_elements_.push_back(ano_tmp);

    for(auto elem: exp->intersects)
      update(elem, ano_tmp, false);
  }
  else
    ano->sub_elements_.push_back(createElement(exp));
}

AnonymousClassBranch_t* AnonymousClassGraph::add(const std::string& value, AnonymousClassVectors_t& ano)
{   
    std::lock_guard<std::shared_timed_mutex> lock(Graph<AnonymousClassBranch_t>::mutex_);

    std::string ano_name = "ano"+ std::to_string(cpt_anonymous_classes_);
    AnonymousClassBranch_t* anonymous_branch = new AnonymousClassBranch_t(ano_name);
    //std::cout << "New anonymous branch id : " << anonymous_branch->value() << " =>" << ano.str_equivalences << std::endl;
    cpt_anonymous_classes_++;

    anonymous_classes_.push_back(anonymous_branch);

    ExpressionMember_t* root = ano.equivalence;
    AnonymousClassElement_t* ano_elem = new AnonymousClassElement_t();

     // Class equivalent to 
    ClassBranch_t* class_branch = class_graph_->findOrCreateBranch(value);
    anonymous_branch->class_equiv_= class_branch;
    class_branch->equiv_relations_.push_back(anonymous_branch);

    if(root->mother == nullptr)
    {
      ExpressionMember_t* leaf = root->intersects.front();

      if(leaf->intersects.size() != 0)
      {
        update(leaf, ano_elem, true);
        // not so good but works to erase the empty node at the beginning
        anonymous_branch->ano_elem_ = ano_elem->sub_elements_.front();
      }
      else
        anonymous_branch->ano_elem_ = createElement(leaf);
    }
  return anonymous_branch;
}



} // namespace ontologenius