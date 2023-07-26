#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"

#include <algorithm>
#include <iostream>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {


AnonymousClassGraph::AnonymousClassGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, 
DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
  individual_graph_ = individual_graph;
}

AnonymousClassBranch_t* AnonymousClassGraph::add( const std::string& value, AnonymousClassVectors_t& ano)
{   
    std::lock_guard<std::shared_timed_mutex> lock(Graph<AnonymousClassBranch_t>::mutex_);

    std::string ano_name = "ano"+ std::to_string(cpt_anonymous_classes_);
    auto anonymous_branch = new AnonymousClassBranch_t(ano_name);
    std::cout << "New anonymous branch id : " << anonymous_branch->value() << " =>" << ano.str_equivalences << std::endl;
    cpt_anonymous_classes_++;

    // Class equivalent to 
    ClassBranch_t* class_branch = class_graph_->findOrCreateBranch(value);
    anonymous_branch->class_equiv_= class_branch;

    
    //if size == 1 => Component Eq to Camera
    // if(ano.equiv_vect[0].size() == 1)
    // {
    //   std::cout << "Equivalence only to a class" << std::endl;

    // }
    // else
    // {
    //   // if last element is a Class then the property is an obj prop, if it is a type (e.g boolean) then it is a data property
    //   // Object property
    //   std::string object_prop_str = ano.equiv_vect[0][0];
    //   ObjectPropertyBranch_t* object_branch = object_property_graph_->findOrCreateBranch(object_prop_str);
    //   anonymous_branch->object_property_involved_= object_branch;

    //   // Data property
    //   std::string data_prop_str = ano.equiv_vect[0][0];
    //   DataPropertyBranch_t* data_branch = data_property_graph_->findOrCreateBranch(data_prop_str);
    //   anonymous_branch->data_property_involved_= data_branch;

    //   // Individuals
    //   std::string indiv_str = ano.equiv_vect[0][2];
    //   IndividualBranch_t* indiv_branch = individual_graph_->findOrCreateBranch(indiv_str);
    //   anonymous_branch->individual_involved_= indiv_branch;
 
    // }
    
    // look for the restriction field :
    // if property is not null
    //-> look in ObjectProperty -> if found -> look for the next part in the ClassBranch
    // -> look in DataProperty -> if found -> look for the next part in the type
    
    // in the equiv relation :

    // look for the ClassBranch_t 
   
    //class_equiv = class_graph_->findBranch(ano);
    // triple check for classes 
    // ClassBranch_t* domain_branch = nullptr;
    // getInMap(&domain_branch, domain.elem, class_graph_->roots_);
    // getInMap(&domain_branch, domain.elem, class_graph_->branchs_);
    // getInMap(&domain_branch, domain.elem, class_graph_->tmp_mothers_);



    // Fill the cardinality


    return new AnonymousClassBranch_t();

}

AnonymousClassGraph::AnonymousClassGraph(const AnonymousClassGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph)
{
  class_graph_ = class_graph;
  individual_graph_ = individual_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
}

} // namespace ontologenius