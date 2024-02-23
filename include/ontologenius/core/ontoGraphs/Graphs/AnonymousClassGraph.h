#ifndef ONTOLOGENIUS_ANOCLASSGRAPH_H
#define ONTOLOGENIUS_ANOCLASSGRAPH_H

#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"

namespace ontologenius {

struct Cardinality_t
{
  std::string cardinality_type;
  std::string cardinality_number;
  std::string cardinality_range;
  
  std::string toString()
  {
    std::string res;
    if(cardinality_type != "")
      res += cardinality_type + " ";
    if(cardinality_number != "")
      res += cardinality_number + " ";
    if(cardinality_range != "")
      res += cardinality_range;
    return res;
  }

  std::vector<std::string> toVector()
  {
    return {cardinality_range, cardinality_number, cardinality_type};
  }
};

struct Restriction_t
{
  Cardinality_t card;
  std::string property;
  std::string restriction_range;

  std::string toString()
  {
    std::string res;

    if(!property.empty())
      res += property + " ";

    res += card.toString();

    if(!restriction_range.empty())
      res += restriction_range;

    return res;
  }

  std::vector<std::string> toVector()
  {
    std::vector<std::string> result;
    if(property != "")
      result.push_back(property);
    if(card.cardinality_type != "")
      result.push_back(card.cardinality_type);
    if(card.cardinality_number != "")
      result.push_back(card.cardinality_number);
    if(card.cardinality_range != "")
      result.push_back(card.cardinality_range);
    if(restriction_range != "")
      result.push_back(restriction_range);

    return result;
  }
};

struct ExpressionMember_t
{
  LogicalNodeType_e logical_type_;
  bool oneof;
  bool is_complex;
  bool is_data_property;

  Restriction_t rest; // Restriction (e.g hasComponent some Camera)
  std::vector<ExpressionMember_t*> child_members;
  ExpressionMember_t* mother;
  std::string ano_name;

  ExpressionMember_t() : logical_type_(logical_none), oneof(false),
                         is_complex(false), is_data_property(false), mother(nullptr), ano_name("") {}  

  std::string toString()
  {
    std::string str_equivalence;

    if(child_members.size() == 0)
      str_equivalence = rest.toString();
    else if(logical_type_ == logical_not)
      str_equivalence = "not (" + child_members.front()->toString() + ")";
    else
    {
      std::string inner;
      for(auto child : child_members)
      {
        if(inner != "")
        {
          if(logical_type_ == logical_and)
            inner += " and ";
          else if(logical_type_ == logical_or)
            inner += " or ";
          else if(oneof == true)
            inner += ", " ;
        }
        if(oneof == true)
          inner += child->toString();
        else
          inner += "(" + child->toString() + ")";
      }

      if(oneof == true)
        str_equivalence = "oneOf (" + inner + ")";
      else if(is_complex)
        str_equivalence = rest.toString() + inner;
      else
        str_equivalence = inner;
    }

    return str_equivalence;
  }
};

struct AnonymousClassVectors_t
{
  std::string class_equiv;
  std::vector<ExpressionMember_t*> equivalence_trees;
  std::vector<std::string> str_equivalences;
};

//for friend
class ObjectPropertyGraph;
class DataPropertyGraph;
class IndividualGraph;
class ClassGraph;

class AnonymousClassChecker;

class AnonymousClassGraph : public Graph<AnonymousClassBranches_t>
{
  friend ObjectPropertyGraph;
  friend DataPropertyGraph;
  friend IndividualGraph;
  friend ClassGraph;

  friend AnonymousClassChecker;
public:
    AnonymousClassGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, 
                        DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
    AnonymousClassGraph(const AnonymousClassGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, 
                        DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
    ~AnonymousClassGraph() {}

    AnonymousClassElement_t* createElement(ExpressionMember_t* exp_leaf);
    void update(ExpressionMember_t* exp, AnonymousClassElement_t* ano_class);
    AnonymousClassElement_t* createTree(ExpressionMember_t* member_node, size_t& depth);
    AnonymousClassBranches_t* add(const std::string& value, AnonymousClassVectors_t& ano_class);
    AnonymousClassElement_t* resolveTree(AnonymousClassElement_t* elem,  bool prev_and);

    void printTree(AnonymousClassElement_t* ano_elem, size_t level, bool root);
    std::string toString(CardType_t value);
    
private:
    ClassGraph* class_graph_;
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;
    IndividualGraph* individual_graph_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOCLASSGRAPH_H