#ifndef ONTOLOGENIUS_ANOCLASSGRAPH_H
#define ONTOLOGENIUS_ANOCLASSGRAPH_H

#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"

namespace ontologenius {


struct Cardinality_t
{
  std::string cardinality_type = "";
  std::string cardinality_number = "";
  std::string cardinality_range = "";
  
  std::string getCardinality(){
    std::string res = " ";
    //std::cout << "type" << cardinality_type + " number " + cardinality_number + " range " + cardinality_range;
    if(cardinality_type != "")
      res += cardinality_type + " ";
    if(cardinality_number != "")
      res += cardinality_number +" ";
    if(cardinality_range != "")
      res += cardinality_range + " ";
    return res;
  }

  std::vector<std::string> getCardinalityVector(){
    std::vector<std::string> result;
    result.push_back(cardinality_range);
    result.push_back(cardinality_number);
    result.push_back(cardinality_type);
    //std::cout << cardinality_type + " " + cardinality_number + " " + cardinality_range;
    return result;
  }
  
};

struct Restriction_t
{
  Cardinality_t card;
  std::string property = "";
  std::string restriction_range = "";

  std::string getRestriction()
  {
    //std::cout << "property : " << property + " " + "cardinality : " << card->getCardinality() + " " + "restriction : " << restriction_range << std::endl;
  std::string res = " ";

  if(!property.empty())
    res +=property +" ";

  res+= card.getCardinality();

  if(!restriction_range.empty())
    res += restriction_range + " ";

  return res;
  }

  std::vector<std::string> getRestrictionVector()
  {
    std::vector<std::string> result;
    //card->cardinality_number.empty();
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

    //std::cout << "property : " << property + " " + "cardinality : " << card->getCardinality() + " " + "restriction : " << restriction_range << std::endl;
    return result;
  }
};

struct ExpressionMember_t{

  LogicalNodeType_e logical_type_ = logical_none;
  bool andor = false; // true = and / false = or
  bool negation = false; // true = not node / false = nothing
  bool oneof = false; // true = OneOf node
  int nb_sub = 0; // number of sub elements
  bool isDataProp = false;
  bool distributable = false; // true if contains "and" & "or" nodes
  Restriction_t rest; // Restriction (e.g hasComponent some Camera)
  std::string class_restriction; // if the restriction is only a class restriction (e.g A Eq to B)
  std::vector<ExpressionMember_t*> intersects; // sub elements
  ExpressionMember_t* mother = nullptr;
  std::string str_equivalence;
  std::string distributed_equivalence;

  ExpressionMember_t() : logical_type_(logical_none), andor(nullptr), negation(nullptr), oneof(nullptr), nb_sub(0), isDataProp(false) {}  

  void UpdateEquiv()
  {
    int current = 0;
    int size_inter = intersects.size();

    str_equivalence = "(";
    if(negation)
      str_equivalence += " not (";

    for(auto elem2: intersects)
    {
      str_equivalence += elem2->str_equivalence;
      if(elem2->mother->oneof)
        str_equivalence += "," ;
      else if(current < size_inter - 1 )
        str_equivalence += (elem2->mother->andor) ? " and " : " or ";
      
      current++;
    }
    str_equivalence += ")";
    
    if(negation)
      str_equivalence += ")";

  }

// Distribution function with strings
  void DistributeEquiv(){

    // R1 or (R2 or R3) => 3 equivalences R1 / R2 / R3
    // R1 and (R2 or R3) => 2 equivalences R1 and R2 / R1 and R3
    // R1 or (R2 and R3) => 2 equivalences R1 / R2 and R3
    // R1 and (R2 and R3) => 1 equivalence R1 and R2 and R3

    bool distrib = false;
    std::vector<std::vector<std::string>> list_or;
    std::vector<std::string> list_2;

    for(auto elem : intersects)
    { // OR node
      if(elem->andor == false && elem->nb_sub > 0)
      {
        std::vector<std::string> list_1;
        //std::cout << "Distributable node" << elem->str_equivalence << std::endl;
        distrib = true;
        for(auto elem2 : elem->intersects)
        {
          list_1.push_back(elem2->str_equivalence);
        }
        list_or.push_back(list_1);
      }
      // restriction node
      else if(elem->andor==false && elem->nb_sub == 0)
      {
        //std::cout << "Restriction " << elem->str_equivalence << std::endl;
        list_2.push_back(elem->str_equivalence);
      }
    }

    if(distrib)
    {
      std::cout << "Distributed expressions : " << std::endl;
       // many to many
      if(list_or.size() == 2)
      {
        std::cout << "two complex members" << std::endl;
        std::vector<std::string> list_complex_left = list_or[0];
        std::vector<std::string> list_complex_right = list_or[1];

        for( auto& value1 : list_complex_left)
        {
          for( auto& value2 : list_complex_right)
          {
            std::cout << value1 + " and " + value2 << "\n";
            distributed_equivalence += "(" + value1 + " and " + value2 +  ")";
            if(&value2 != &list_complex_right.back())
            {
              distributed_equivalence += " or ";
            }
          }
          if(&value1 != &list_complex_left.back())
          {
              distributed_equivalence += " or ";
            }
        }
        std::cout <<" NEW equivalence is : " <<  distributed_equivalence << std::endl;
      }
      // one to many
      else
      {
        std::cout << "one complex member" << std::endl;
        std::vector<std::string> list_complex_left = list_or[0];
        for( auto& value1: list_complex_left)
        {
          for( auto& value2: list_2)
          {
            std::cout << value1 + " and " + value2 << "\n";
            distributed_equivalence += "(" + value1 + " and " + value2 +  ")";
            if (&value1 != &list_complex_left.back()){
              distributed_equivalence += " or ";
            }
          }
        }
        std::cout <<" NEW equivalence is : " <<  distributed_equivalence << std::endl;
      }
    }
  }

// Distribution function with expression members and transformation of the tree
  void DistributeEquiv_v2(){

    // R1 or (R2 or R3) => 3 equivalences R1 / R2 / R3
    // R1 and (R2 or R3) => 2 equivalences R1 and R2 / R1 and R3
    // R1 or (R2 and R3) => 2 equivalences R1 / R2 and R3
    // R1 and (R2 and R3) => 1 equivalence R1 and R2 and R3

    bool distrib = false;
    std::vector<std::vector<ExpressionMember_t*>> list_children;
    std::vector<ExpressionMember_t*> list_restrictions;
    
    for(auto elem : intersects)
    { // OR node
      if(elem->andor == false && elem->nb_sub > 0)
      {
        std::vector<ExpressionMember_t*> list_1;
        //std::cout << "Distributable node" << elem->str_equivalence << std::endl;
        distrib = true;
        for(auto elem2 : elem->intersects)
        {
          list_1.push_back(elem2);
        }
        list_children.push_back(list_1);
      }
      // restriction node
      else if(elem->andor==false && elem->nb_sub == 0)
      {
        //std::cout << "Restriction " << elem->str_equivalence << std::endl;
        list_restrictions.push_back(elem);
      }
    }

    if(distrib)
    {
      // erase the content of the And node intersections and transform it into an or node
      this->intersects.clear();
      this->andor=false;
      
      // std::cout << "Distributed expressions : " << std::endl;
      // many to many

      if(list_children.size() == 2)
      {
        //std::cout << "two complex members" << std::endl;
        std::vector<ExpressionMember_t*> list_complex_left = list_children[0];
        std::vector<ExpressionMember_t*> list_complex_right = list_children[1];

        for( auto& value1 : list_complex_left)
        {

          for(auto& value2 : list_complex_right)
          {
            // creation of an AND node
            ExpressionMember_t* exp2 = new ExpressionMember_t;
            exp2->mother = this;
            exp2->andor = true;
            this->intersects.push_back(exp2);
            // push back the sub elements of the newly created and node
            value1->mother=exp2;
            value2->mother=exp2;
          
            exp2->intersects.push_back(value1);
            exp2->intersects.push_back(value2);
            exp2->UpdateEquiv();
          }
        }
      }
      // one to many
      else
      {
        //std::cout << "one complex member" << std::endl;
        std::vector<ExpressionMember_t*> list_complex_left = list_children[0];
        
        for( auto& value1: list_complex_left)
        { 
          ExpressionMember_t* exp2 = new ExpressionMember_t;
          exp2->mother = this;
          exp2->andor = true;
          this->intersects.push_back(exp2);
          for( auto& value2: list_restrictions)
          {
            value1->mother=exp2;
            value2->mother=exp2;

            exp2->intersects.push_back(value1);
            exp2->intersects.push_back(value2);
            exp2->UpdateEquiv();
          }
        }
      }
      this->UpdateEquiv();
    }
  }
};

struct AnonymousClassVectors_t
{
  std::string class_equiv;
  ExpressionMember_t* equivalence;
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
    AnonymousClassElement_t* createElement(ExpressionMember_t* exp_leaf);
    void update(ExpressionMember_t* exp, AnonymousClassElement_t* ano_class, bool root);
    AnonymousClassBranch_t* add(const std::string& value, AnonymousClassVectors_t& ano_class);
    
    void close() {};
    std::vector<AnonymousClassBranch_t*> get() override { return anonymous_classes_;}
    
private:
    ClassGraph* class_graph_;
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;
    IndividualGraph* individual_graph_;
    int cpt_anonymous_classes_;
    std::vector<AnonymousClassBranch_t*> anonymous_classes_;

};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOCLASSGRAPH_H