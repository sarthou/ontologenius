#ifndef ONTOLOGENIUS_ANOCLASSGRAPH_H
#define ONTOLOGENIUS_ANOCLASSGRAPH_H

#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"
#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"

namespace ontologenius {

//for friend

class ObjectPropertyGraph;
class DataPropertyGraph;
class IndividualGraph;
class ClassGraph;


// OntoGraph and not just Graph because in Ontology.h -> need to have only arguments which have a get() and close() specification
// class AnonymousClassGraph : public OntoGraph<AnonymousClassBranch_t>
class AnonymousClassGraph : public Graph<AnonymousClassBranch_t>
{

friend ObjectPropertyGraph;
friend DataPropertyGraph;
friend IndividualGraph;
friend ClassGraph;

public:
    AnonymousClassGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, 
    DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
    AnonymousClassGraph(const AnonymousClassGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, 
    DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
    ~AnonymousClassGraph() {}; 
    AnonymousClassBranch_t* add(const std::string& value, AnonymousClassVectors_t& ano_class);

    void close() {};
    std::vector<AnonymousClassBranch_t*> get() override { return anonymous_classes_;}
    
private:
    ClassGraph* class_graph_;
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;
    IndividualGraph* individual_graph_;
    // ex : ano_001 en string obligé !
    int cpt_anonymous_classes_;
    std::vector<AnonymousClassBranch_t*> anonymous_classes_;        // contains all the anonymous_classes
    // std::unordered_map<std::string, AnonymousClassBranch_t>
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOCLASSGRAPH_H