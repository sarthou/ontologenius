#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"

#include <algorithm>
#include <iostream>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

#include "ontologenius/utils/String.h"

//#define DEBUG

namespace ontologenius {

AnonymousClassGraph::AnonymousClassGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, 
                                        DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
  individual_graph_ = individual_graph;
}

AnonymousClassGraph::AnonymousClassGraph(const AnonymousClassGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, 
                                        DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph)
{
  class_graph_ = class_graph;
  individual_graph_ = individual_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;

  for(auto branch : other.all_branchs_)
  {
    auto class_branch = new AnonymousClassBranches_t(branch->value());
    all_branchs_.push_back(class_branch);
  }
}

AnonymousClassElement_t* AnonymousClassGraph::createElement(ExpressionMember_t* exp)
{
  AnonymousClassElement_t* ano_element = new AnonymousClassElement_t();
  std::vector<std::string> vect_equiv = exp->rest.toVector();
  ano_element->is_complex = exp->is_complex;

  if(exp->logical_type_ != logical_none)
  {
    ano_element->logical_type_ = exp->logical_type_;
    return ano_element;
  }
  else if(exp->oneof == true)
  {
    ano_element->oneof = true;
    return ano_element;
  }
  // class, indiv or literal node only
  if(vect_equiv.size() == 1)
  {
    std::string equiv = vect_equiv.front();
    if(isIn("http://www.w3.org/", equiv))
    {
      std::string type = split(equiv, "#").back();
      LiteralNode* literal = data_property_graph_->createLiteral(type + "#");
      ano_element->card_.card_range_ = literal;
    }
    else if(exp->mother != nullptr && exp->mother->oneof)
      ano_element->individual_involved_= individual_graph_->findOrCreateBranch(equiv);
    else
      ano_element->class_involved_= class_graph_->findOrCreateBranch(equiv);
  }
  else if(vect_equiv[1] == "value")
  { 
    ano_element->object_property_involved_= object_property_graph_->findOrCreateBranch(vect_equiv.front());
    ano_element->card_.card_type_ = cardinality_value;
    ano_element->individual_involved_= individual_graph_->findOrCreateBranch(vect_equiv[2]);
  }
  else
  {
    std::string property = vect_equiv.front();

    if(vect_equiv[1]=="some")
      ano_element->card_.card_type_ = cardinality_some;
    else if(vect_equiv[1]=="only")
      ano_element->card_.card_type_ = cardinality_only;
    else if(vect_equiv[1]=="exactly")
      ano_element->card_.card_type_ = cardinality_exactly;
    else if(vect_equiv[1]=="min")
      ano_element->card_.card_type_ = cardinality_min;
    else if(vect_equiv[1]=="max")
      ano_element->card_.card_type_ = cardinality_max;
    else
      ano_element->card_.card_type_ = cardinality_error;

    if(exp->is_complex)
    {
      if(exp->is_data_property)
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

      if(isIn("http://www.w3.org/", vect_equiv.back()))
      {
        ano_element->data_property_involved_= data_property_graph_->findOrCreateBranch(property);
        std::string type = split(vect_equiv.back(), "#").back();
        LiteralNode* literal = data_property_graph_->createLiteral(type + "#");
        ano_element->card_.card_range_ = literal;
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

AnonymousClassElement_t* AnonymousClassGraph::createTree(ExpressionMember_t* member_node, size_t& depth)
{
  size_t local_depth = depth + 1;
  AnonymousClassElement_t* node = createElement(member_node);
  
  for(auto child : member_node->child_members)
  {
    size_t child_depth = depth +1;
    node->sub_elements_.push_back(createTree(child, child_depth));
    if(child_depth > local_depth)
      local_depth = child_depth;
  }

  depth = local_depth;

  return node;
}

AnonymousClassBranches_t* AnonymousClassGraph::add(const std::string& value, AnonymousClassVectors_t& ano)
{   
  std::lock_guard<std::shared_timed_mutex> lock(Graph<AnonymousClassBranches_t>::mutex_);

  std::string ano_name = "anonymous_"+ value;
  AnonymousClassBranches_t* anonymous_branch = new AnonymousClassBranches_t(ano_name);
  ClassBranch_t* class_branch = class_graph_->findOrCreateBranch(value);

  anonymous_branch->class_equiv_= class_branch;
  all_branchs_.push_back(anonymous_branch);
  class_branch->equiv_relations_ = anonymous_branch;
  
  for(size_t i = 0 ; i < ano.equivalence_trees.size() ; i++)
  {
    size_t depth = 0;
    AnonymousClassElement_t* ano_elem = createTree(ano.equivalence_trees[i], depth);
    ano_elem->ano_name = ano_name + "_" + std::to_string(i);
    anonymous_branch->ano_elems_.push_back(ano_elem);
    anonymous_branch->depth_ = depth;

    #ifdef DEBUG
    printTree(ano_elem, 3, true);
    #endif
  }

  return anonymous_branch;
}

void AnonymousClassGraph::printTree(AnonymousClassElement_t* ano_elem, size_t level, bool root)
{ 
  std::string space(level*4, ' ');
  std::string tmp = "";

  if(root)
    std::cout << space;

  if(ano_elem->logical_type_ == LogicalNodeType_e::logical_and)
    tmp+= "and";
  else if(ano_elem->logical_type_ == LogicalNodeType_e::logical_or)
    tmp+= "or";
  else if(ano_elem->logical_type_ == LogicalNodeType_e::logical_not)
    tmp+= "not";
  else if(ano_elem->oneof)
    tmp+= "oneOf";
  else if(ano_elem->object_property_involved_ != nullptr)
  {
    tmp += ano_elem->object_property_involved_->value();
    tmp += " " + toString(ano_elem->card_.card_type_);

    if(ano_elem->card_.card_type_ == ontologenius::CardType_t::cardinality_value)
      tmp += " " + ano_elem->individual_involved_->value();
    else
    {
      if(ano_elem->card_.card_number_ != 0)
        tmp += " " +  std::to_string(ano_elem->card_.card_number_);
      if(ano_elem->class_involved_ != nullptr)
        tmp += " " + ano_elem->class_involved_->value();
    }
  }
  else if(ano_elem->data_property_involved_ != nullptr)
  {
    tmp += ano_elem->data_property_involved_->value();
    tmp += " " + toString(ano_elem->card_.card_type_);
    if(ano_elem->card_.card_number_ != 0)
      tmp += " " +  std::to_string(ano_elem->card_.card_number_);
    if(ano_elem->card_.card_range_ != nullptr)
      tmp += " " + ano_elem->card_.card_range_->value();
  }
  else
  { 
    if(ano_elem->class_involved_ != nullptr)
      tmp += ano_elem->class_involved_->value();
    else if(ano_elem->individual_involved_ != nullptr)
      tmp += ano_elem->individual_involved_->value();
    else if(ano_elem->card_.card_range_ != nullptr)
      tmp += ano_elem->card_.card_range_->type_;
  }

  std::cout << tmp << std::endl;

  for(auto sub_elem : ano_elem->sub_elements_)
  {
    for(int i = 0; i < int(level) ; i++)
      std::cout << "│   ";
    if(sub_elem == ano_elem->sub_elements_.back())
      std::cout << "└── ";
    else
       std::cout << "├── " ;    
    printTree(sub_elem, level + 1, false);
  }
}

std::string AnonymousClassGraph::toString(CardType_t value)
{
  switch (value)
  {
  case CardType_t::cardinality_some:
    return "some";
  case CardType_t::cardinality_only:
    return "only";
  case CardType_t::cardinality_min:
    return "min";
  case CardType_t::cardinality_max:
   return "max";
  case CardType_t::cardinality_exactly:
    return "exactly";
  case CardType_t::cardinality_value:
    return "value";
  case CardType_t::cardinality_error:
    return "error";
  default:
    return "";
  }
}

} // namespace ontologenius