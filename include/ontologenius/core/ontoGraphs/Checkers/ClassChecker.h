#ifndef ONTOLOGENIUS_CLASSCHECKER_H
#define ONTOLOGENIUS_CLASSCHECKER_H

#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"

namespace ontologenius {

  class ClassChecker : public ValidityChecker<ClassBranch>
  {
  public:
    explicit ClassChecker(ClassGraph* graph) : ValidityChecker(graph) { class_graph_ = graph; }
    ~ClassChecker() = default;

    size_t check() override;
    void printStatus() { ValidityChecker<ClassBranch>::printStatus("class", "classes", graph_vect_.size()); }

  private:
    void checkDisjoint(ClassBranch* branch, std::unordered_set<ClassBranch*> up);

    void checkObjectPropertyDomain(ClassBranch* branch, std::unordered_set<ClassBranch*> up);
    void checkObjectPropertyRange(ClassBranch* branch);

    void checkDataPropertyDomain(ClassBranch* branch, std::unordered_set<ClassBranch*> up);
    void checkDataPropertyRange(ClassBranch* branch);

    ClassGraph* class_graph_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSCHECKER_H
