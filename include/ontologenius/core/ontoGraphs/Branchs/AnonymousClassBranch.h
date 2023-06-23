#ifndef ONTOLOGENIUS_ANOBRANCH_H
#define ONTOLOGENIUS_ANOBRANCH_H

#include <string>
#include <vector>

namespace ontologenius {

struct Cardinality_t
{
  std::string cardinality_type;
  std::string cardinality_number;
  std::string cardinality_range;
  
  std::string getCardinality(){
    //std::cout << cardinality_type + " " + cardinality_number + " " + cardinality_range;
    return cardinality_type + " " + cardinality_number + " " + cardinality_range;
  }
  
};

struct Restriction_t
{
  Cardinality_t* card;
  std::string property;
  std::string restriction_range;

  std::string getRestriction()
  {
    //std::cout << property + " " + card->getCardinality() + " " + restriction_range;
    return property + " " + card->getCardinality() + " " + restriction_range;
  }
};

struct ExpressionMember_t{

  bool andor; // true = and / false = or
  int nb_sub; // number of sub elements
  bool distributable; // true if contains "and" & "or" nodes
  Restriction_t rest; // Restriction (e.g hasComponent some Camera)
  std::string class_restriction; // if the restriction is only a class restriction (e.g A Eq to B)
  std::vector<ExpressionMember_t*> intersects; // sub elements
  ExpressionMember_t* mother;
  std::string str_equivalence;

  void UpdateEquiv()
  {
    int current = 0;
    int size_inter = intersects.size();

    str_equivalence = "(";

    for(auto elem2: intersects)
    {
      str_equivalence += elem2->str_equivalence;
      if(current < size_inter - 1 )
        str_equivalence += (elem2->mother->andor) ? " and " : " or ";
      current++;
    }
    str_equivalence += ")";
  }

  // void DistributeEquiv(){
  //   std::string new_exp;
  //   std::vector<std::string> list_simple;
  //   std::vector<std::string> list_complex;

  //   //enter here only if precedent node is an and node
  //   for(auto elem: intersects){ // loop over its children

  //     //The or node
  //     if(elem->andor==false && elem->nb_sub > 0){ // differentiating conditions from leaf node and or node
  //       std::cout << "distributable : " << elem->str_equivalence << std::endl;

  //       for(auto elem2 :elem->intersects){
  //         std::cout << elem2->str_equivalence << std::endl;
    
  //       }

      


  //       for
  //       // new_exp = elem->str_equivalence;

  //       // for(auto elem2 :elem->intersects){
  //       //   std::cout << elem2->str_equivalence << std::endl;
          
  //       //   new_exp+= " or ";
  //       //   new_exp+= elem2->str_equivalence;
  //       //   std::cout << new_exp << std::endl;
  //       // }
        
  //     }
  //   }
  // }

};

struct AnonymousClass_t
{
  std::string id;
  std::string class_equiv;
  ExpressionMember_t* equivalence;
  std::string str_equivalences;
  
};
} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOBRANCH_H