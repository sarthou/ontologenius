#ifndef ONTOLOGENIUS_OBJECTROPERTYGRAPH_H
#define ONTOLOGENIUS_OBJECTROPERTYGRAPH_H

#include <cstddef>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"

namespace ontologenius {

  struct ObjectPropertyVectors_t
  {
    std::vector<SingleElement<std::string>> mothers_;
    std::vector<SingleElement<std::string>> disjoints_;
    std::vector<SingleElement<std::string>> inverses_;
    std::vector<SingleElement<std::string>> domains_;
    std::vector<SingleElement<std::string>> ranges_;
    std::vector<std::vector<std::string>> chains_;
    Properties_t properties_;
    std::map<std::string, std::vector<std::string>> dictionary_;
    std::map<std::string, std::vector<std::string>> muted_dictionary_;
    bool annotation_usage_;

    ObjectPropertyVectors_t() : annotation_usage_(false) {}
  };

  // for friend
  class ObjectPropertyDrawer;
  class IndividualGraph;
  class AnonymousClassGraph;

  // for graphs usage
  class ClassGraph;

  class ObjectPropertyGraph : public OntoGraph<ObjectPropertyBranch>
  {
    friend ObjectPropertyDrawer;
    friend IndividualGraph;
    friend ClassGraph;
    friend AnonymousClassGraph;

  public:
    explicit ObjectPropertyGraph(IndividualGraph* individual_graph, ClassGraph* class_graph);
    ObjectPropertyGraph(const ObjectPropertyGraph& other, IndividualGraph* individual_graph, ClassGraph* class_graph);
    ~ObjectPropertyGraph() override = default;

    void deepCopy(const ObjectPropertyGraph& other);

    ObjectPropertyBranch* add(const std::string& value, ObjectPropertyVectors_t& property_vectors);
    void add(std::vector<std::string>& disjoints);

    std::unordered_set<std::string> getInverse(const std::string& value);
    std::unordered_set<index_t> getInverse(index_t value);
    std::unordered_set<std::string> getDomain(const std::string& value, size_t depth = -1);
    std::unordered_set<index_t> getDomain(index_t value, size_t depth = -1);
    void getDomainPtr(ObjectPropertyBranch* branch, std::unordered_set<ClassBranch*>& res, size_t depth = -1);
    std::unordered_set<std::string> getRange(const std::string& value, size_t depth = -1);
    std::unordered_set<index_t> getRange(index_t value, size_t depth = -1);
    void getRangePtr(ObjectPropertyBranch* branch, std::unordered_set<ClassBranch*>& res, size_t depth = -1);
    void getDomainAndRangePtr(ObjectPropertyBranch* branch, std::unordered_set<ClassBranch*>& domains, std::unordered_set<ClassBranch*>& ranges, size_t depth = -1);

    bool addInverseOf(const std::string& from, const std::string& on);
    bool removeInverseOf(const std::string& from, const std::string& on);

    bool isIrreflexive(const std::string& prop);
    bool isIrreflexive(ObjectPropertyBranch* prop);
    bool isAsymetric(const std::string& prop);
    bool isAsymetric(ObjectPropertyBranch* prop);

  private:
    ClassGraph* class_graph_;

    template<typename T>
    void getDomain(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<T>& res, std::unordered_set<ObjectPropertyBranch*>& up_trace);
    void getDomainPtr(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<ClassBranch*>& res, std::unordered_set<ObjectPropertyBranch*>& up_trace);
    template<typename T>
    void getRange(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<T>& res, std::unordered_set<ObjectPropertyBranch*>& up_trace);
    void getRangePtr(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<ClassBranch*>& res, std::unordered_set<ObjectPropertyBranch*>& up_trace);
    void getDomainAndRangePtr(ObjectPropertyBranch* branch, size_t depth, std::unordered_set<ClassBranch*>& domains, std::unordered_set<ClassBranch*>& ranges, std::unordered_set<ObjectPropertyBranch*>& up_trace);

    void cpyBranch(ObjectPropertyBranch* old_branch, ObjectPropertyBranch* new_branch);
    void cpyChainOfBranch(ObjectPropertyBranch* old_branch, ObjectPropertyBranch* new_branch);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJECTROPERTYGRAPH_H
