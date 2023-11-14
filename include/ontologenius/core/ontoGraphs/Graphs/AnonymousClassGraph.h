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
    std::string res = " ";
    if(cardinality_type != "")
      res += cardinality_type + " ";
    if(cardinality_number != "")
      res += cardinality_number +" ";
    if(cardinality_range != "")
      res += cardinality_range + " ";
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
    std::string res = " ";

    if(!property.empty())
      res += property +" ";

    res += card.toString();

    if(!restriction_range.empty())
      res += restriction_range + " ";

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
  bool oneof; // true = OneOf node
  bool is_complex; // number of sub elements
  bool is_data_property;

  Restriction_t rest; // Restriction (e.g hasComponent some Camera)
  std::vector<ExpressionMember_t*> intersects; // sub elements
  ExpressionMember_t* mother;
  std::string str_equivalence;
  std::string distributed_equivalence;

  ExpressionMember_t() : logical_type_(logical_none), oneof(false),
                         is_complex(false), is_data_property(false), mother(nullptr) {}  

  void updateEquivString()
  {
    int current = 0;
    int size_inter = intersects.size();

    str_equivalence = "(";
    if(logical_type_ == logical_not)
      str_equivalence += " not (";

    for(auto elem2 : intersects)
    {
      str_equivalence += elem2->str_equivalence;
      if(elem2->mother->oneof)
        str_equivalence += "," ;
      else if(current < size_inter - 1 )
      {
        if(logical_type_ == logical_and)
          str_equivalence += " and ";
        else if(logical_type_ == logical_or)
          str_equivalence += " or ";
      }
      
      current++;
    }
    str_equivalence += ")";
    
    if(logical_type_ == logical_not)
      str_equivalence += ")";
  }
};

struct AnonymousClassVectors_t
{
  std::string class_equiv;
  ExpressionMember_t* equivalence_tree;
  std::string str_equivalences;
  std::vector<std::vector<std::string>> equiv_vect;
};

//for friend
class ObjectPropertyGraph;
class DataPropertyGraph;
class IndividualGraph;
class ClassGraph;

class AnonymousClassChecker;

// OntoGraph and not just Graph because in Ontology.h -> need to have only arguments which have a get() and close() specification
// class AnonymousClassGraph : public OntoGraph<AnonymousClassBranch_t>
class AnonymousClassGraph : public Graph<AnonymousClassBranch_t>
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
    ~AnonymousClassGraph() {};

    void close() {};
    std::vector<AnonymousClassBranch_t*> get() override { return anonymous_classes_;}

    AnonymousClassElement_t* createElement(ExpressionMember_t* exp_leaf);
    void update(ExpressionMember_t* exp, AnonymousClassElement_t* ano_class);
    AnonymousClassBranch_t* add(const std::string& value, AnonymousClassVectors_t& ano_class);
    AnonymousClassElement_t* resolveTree(AnonymousClassElement_t* elem,  bool prev_and);

    void distributeAnonymousClass(AnonymousClassBranch_t* ano_class_branch);
    void organizeAnonymousClass(AnonymousClassElement_t* ano_class_branch);

    void printTree(AnonymousClassElement_t* ano_elem, size_t level, bool root);
    std::string toString(CardType_t value);
    
private:
    ClassGraph* class_graph_;
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;
    IndividualGraph* individual_graph_;

    std::vector<AnonymousClassBranch_t*> anonymous_classes_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOCLASSGRAPH_H