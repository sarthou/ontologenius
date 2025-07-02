#ifndef ONTOLOGENIUS_DATAPROPERTYGRAPH_H
#define ONTOLOGENIUS_DATAPROPERTYGRAPH_H

#include <cstddef>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"

namespace ontologenius {

  struct DataPropertyVectors_t
  {
    std::vector<SingleElement<std::string>> mothers_;
    std::vector<SingleElement<std::string>> disjoints_;
    std::vector<SingleElement<std::string>> domains_;
    std::vector<std::string> ranges_;
    Properties_t properties_;
    std::map<std::string, std::vector<std::string>> dictionary_;
    std::map<std::string, std::vector<std::string>> muted_dictionary_;
    bool annotation_usage_;

    DataPropertyVectors_t() : annotation_usage_(false) {}
  };

  // for friend
  class DataPropertyDrawer;
  class IndividualGraph;
  class AnonymousClassGraph;

  // for graphs usage
  class ClassGraph;

  class DataPropertyGraph : public OntoGraph<DataPropertyBranch>
  {
    friend DataPropertyDrawer;
    friend IndividualGraph;
    friend ClassGraph;
    friend AnonymousClassGraph;

  public:
    explicit DataPropertyGraph(IndividualGraph* individual_graph, ClassGraph* class_graph);
    DataPropertyGraph(const DataPropertyGraph& other, IndividualGraph* individual_graph, ClassGraph* class_graph);
    ~DataPropertyGraph() override = default;

    void deepCopy(const DataPropertyGraph& other);

    DataPropertyBranch* add(const std::string& value, DataPropertyVectors_t& property_vectors);
    void add(std::vector<std::string>& disjoints);
    bool addAnnotation(const std::string& value, DataPropertyVectors_t& property_vectors);
    LiteralNode* createLiteral(const std::string& value);
    LiteralNode* createLiteralUnsafe(const std::string& value);

    std::unordered_set<std::string> getDomain(const std::string& value, size_t depth = -1);
    std::unordered_set<index_t> getDomain(index_t value, size_t depth = -1);
    void getDomainPtr(DataPropertyBranch* branch, std::unordered_set<ClassBranch*>& res, size_t depth = -1);
    std::unordered_set<std::string> getRange(const std::string& value);
    std::unordered_set<index_t> getRange(index_t value);

    index_t getLiteralIndex(const std::string& name);
    std::vector<index_t> getLiteralIndexes(const std::vector<std::string>& names);
    std::string getLiteralIdentifier(index_t index);
    std::vector<std::string> getLiteralIdentifiers(const std::vector<index_t>& indexes);

  private:
    ClassGraph* class_graph_;
    BranchContainerSet<LiteralNode> literal_container_;

    template<typename T>
    void getDomain(DataPropertyBranch* branch, size_t depth, std::unordered_set<T>& res, std::unordered_set<DataPropertyBranch*>& up_trace);
    void getDomainPtr(DataPropertyBranch* branch, size_t depth, std::unordered_set<ClassBranch*>& res, std::unordered_set<DataPropertyBranch*>& up_trace);

    void cpyBranch(DataPropertyBranch* old_branch, DataPropertyBranch* new_branch);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTYGRAPH_H
