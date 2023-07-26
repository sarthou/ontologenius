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

    for(auto elem : ano.equiv_vect)
    {
      if(elem.size() == 1)
      {
        std::cout << "Class expression equivalence" << std::endl;
        ClassBranch_t* class_branch = class_graph_->findOrCreateBranch(elem[0]);
        anonymous_branch->class_involved_= class_branch;
      }
      else if(elem[1] == "value")
      {
        std::cout << "Value expression equivalence" << std::endl;
        // ObjectProperty
        std::string object_prop_str = elem[0];
        ObjectPropertyBranch_t* object_branch = object_property_graph_->findOrCreateBranch(object_prop_str);
        anonymous_branch->object_property_involved_= object_branch;

        // Individual
        std::string indiv_str = elem[2];
        IndividualBranch_t* indiv_branch = individual_graph_->findOrCreateBranch(indiv_str);
        anonymous_branch->individual_involved_= indiv_branch;
        //Cardinality
        anonymous_branch->card_type_ = value_;

      }
      else if(isIn("http://www.w3.org/", elem.back() ) )
      {
        std::cout << "Data property expression equivalence" << std::endl;
        // Data property
        std::string data_prop_str = elem[0];
        DataPropertyBranch_t* data_branch = data_property_graph_->findOrCreateBranch(data_prop_str);
        anonymous_branch->data_property_involved_= data_branch;
        // Range
        std::string type = split(elem.back(), "#").back();
        anonymous_branch->card_range_ = new LiteralNode(type, "");
        //Cardinality
        if(elem.size() == 4)
          anonymous_branch->card_number_ = stoi(elem[2]);
        else
          anonymous_branch->card_number_ = 0;

        if(elem[1]=="some")
          anonymous_branch->card_type_ = some_;
        else if(elem[1]=="only")
          anonymous_branch->card_type_ = only_;
        else if(elem[1]=="exactly")
          anonymous_branch->card_type_ = exactly_;
        else if(elem[1]=="min")
          anonymous_branch->card_type_ = min_;
        else
          anonymous_branch->card_type_ = max_;
      }
      else{
        std::cout << "Object property expression equivalence" << std::endl;
        // if last element is a Class then the property is an obj prop, if it is a type (e.g boolean) then it is a data property
        // Object property
        std::string object_prop_str = elem[0];
        ObjectPropertyBranch_t* object_branch = object_property_graph_->findOrCreateBranch(object_prop_str);
        anonymous_branch->object_property_involved_= object_branch;
        // Range
        ClassBranch_t* class_branch = class_graph_->findOrCreateBranch(elem.back());
        anonymous_branch->class_involved_= class_branch;
        //Cardinality
        if(elem.size() == 4)
          anonymous_branch->card_number_ = stoi(elem[2]);
        else
          anonymous_branch->card_number_ = 0;

        if(elem[1]=="some")
          anonymous_branch->card_type_ = some_;
        else if(elem[1]=="only")
          anonymous_branch->card_type_ = only_;
        else if(elem[1]=="exactly")
          anonymous_branch->card_type_ = exactly_;
        else if(elem[1]=="min")
          anonymous_branch->card_type_ = min_;
        else
          anonymous_branch->card_type_ = max_;
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