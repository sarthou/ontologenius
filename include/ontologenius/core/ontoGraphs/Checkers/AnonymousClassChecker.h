#ifndef ONTOLOGENIUS_ANONYMOUSCLASSCHECKER_H
#define ONTOLOGENIUS_ANONYMOUSCLASSCHECKER_H

#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"

namespace ontologenius {

  class AnonymousClassChecker : public ValidityChecker<AnonymousClassBranch>
  {
  public:
    explicit AnonymousClassChecker(AnonymousClassGraph* graph) : ValidityChecker(graph) { ano_class_graph_ = graph; }
    ~AnonymousClassChecker() override = default;

    size_t check() override;
    void checkDisjoint();

    void printStatus() override
    {
      ValidityChecker<AnonymousClassBranch>::printStatus("anonymous_class", "anonymous_classes", graph_vect_.size());
    }

  private:
    AnonymousClassGraph* ano_class_graph_;
    std::string current_ano_;

    std::vector<std::string> resolveTree(AnonymousClassElement* ano_elem, const std::vector<ClassElement>& ranges);
    std::vector<std::string> resolveTreeDataTypes(AnonymousClassElement* ano_elem); // Err

    std::vector<std::string> checkPropertyDisjointness(AnonymousClassElement* ano_elem, const std::vector<ClassElement>& ranges);
    void checkIntersectionDomainsDisjointess(AnonymousClassElement* ano_elem);
    std::vector<std::string> checkRangeDomainDisjointness(AnonymousClassElement* ano_elem, const std::vector<ClassElement>& ranges);

    std::vector<std::string> checkExpressionDisjointess(AnonymousClassElement* ano_elem, const std::vector<ClassElement>& ranges);
    void checkObjectPropertyRangeDisjointness(AnonymousClassElement* ano_elem);
    void checkDataPropertyRangeDisjointness(AnonymousClassElement* ano_elem);

    std::string checkClassesDisjointness(ClassBranch* class_left, ClassBranch* class_right); // Err
    std::vector<std::string> checkClassesVectorDisjointness(const std::vector<ClassElement>& classes_left, const std::vector<ClassElement>& class_right);

    template<typename T>
    std::vector<std::string> checkPropertyDomainDisjointness(const T& property, const std::vector<ClassElement>& ranges)
    {
      return checkClassesVectorDisjointness(property->domains_, ranges);
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANONYMOUSCLASSCHECKER_H