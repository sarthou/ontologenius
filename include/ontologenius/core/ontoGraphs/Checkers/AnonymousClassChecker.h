#ifndef ONTOLOGENIUS_ANOCLASSCHECKER_H
#define ONTOLOGENIUS_ANOCLASSCHECKER_H

#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"

// check if the range of each property is disjoint with the domain of the children nodes
// if the domains of elements at the same level are disjoint (i.e no intersection possible), do we reject it ?

namespace ontologenius {

class AnonymousClassChecker : public ValidityChecker<AnonymousClassBranch_t>
{
public:
    explicit AnonymousClassChecker(AnonymousClassGraph* graph) : ValidityChecker(graph) {ano_class_graph_ = graph;}
    ~AnonymousClassChecker() {}

    size_t check() override;
    void checkDisjoint();
    void checkClassesDisjointness(ClassBranch_t* class_left, ClassBranch_t* class_right);
    void compareDomainRange(AnonymousClassElement_t* node, std::vector<ClassElement_t> mother_ranges);

    void printStatus()
    {
        ValidityChecker<AnonymousClassBranch_t>::printStatus("anonymous_class", "anonymous_classes", graph_vect_.size());
    };

private:
    AnonymousClassGraph* ano_class_graph_;

    



};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANOCLASSCHECKER_H