#ifndef ONTOLOGENIUS_ANONYMOUSCLASSCHECKER_H
#define ONTOLOGENIUS_ANONYMOUSCLASSCHECKER_H

#include "ontologenius/core/ontoGraphs/Checkers/ValidityChecker.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"


namespace ontologenius {

class AnonymousClassChecker : public ValidityChecker<AnonymousClassBranches_t>
{
public:
  explicit AnonymousClassChecker(AnonymousClassGraph* graph) : ValidityChecker(graph) {ano_class_graph_ = graph;}
  ~AnonymousClassChecker() {}

  size_t check() override;
  void checkDisjoint();

  void printStatus()
  {
      ValidityChecker<AnonymousClassBranches_t>::printStatus("anonymous_class", "anonymous_classes", graph_vect_.size());
  };

private:
  AnonymousClassGraph* ano_class_graph_;
  std::string current_ano_;

  std::vector<std::string> resolveTree(AnonymousClassElement_t* ano_elem, const std::vector<ClassElement_t>& ranges);
  std::vector<std::string> resolveTreeDataTypes(AnonymousClassElement_t* ano_elem); // Err

  std::vector<std::string> checkPropertyDisjointness(AnonymousClassElement_t* ano_elem, const std::vector<ClassElement_t>& ranges);
  void checkIntersectionDomainsDisjointess(AnonymousClassElement_t* ano_elem);
  std::vector<std::string> checkRangeDomainDisjointness(AnonymousClassElement_t* ano_elem, const std::vector<ClassElement_t>& ranges);

  std::vector<std::string> checkExpressionDisjointess(AnonymousClassElement_t* ano_elem, const std::vector<ClassElement_t>& ranges);
  void checkObjectPropertyRangeDisjointness(AnonymousClassElement_t* ano_elem);
  void checkDataPropertyRangeDisjointness(AnonymousClassElement_t* ano_elem);
  
  std::string checkClassesDisjointness(ClassBranch_t* class_left, ClassBranch_t* class_right); // Err
  std::vector<std::string> checkClassesVectorDisjointness(const std::vector<ClassElement_t>& classes_left, const std::vector<ClassElement_t>& class_right);

  template<typename T>
  std::vector<std::string> checkPropertyDomainDisjointness(const T& property, const std::vector<ClassElement_t>& ranges)
  {
    return checkClassesVectorDisjointness(property->domains_, ranges);
  }

};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANONYMOUSCLASSCHECKER_H