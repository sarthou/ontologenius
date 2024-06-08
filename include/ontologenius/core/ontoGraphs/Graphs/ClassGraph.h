#ifndef ONTOLOGENIUS_CLASSGRAPH_H
#define ONTOLOGENIUS_CLASSGRAPH_H

#include <cstdint>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntoGraph.h"

namespace ontologenius {

  struct ObjectVectors_t
  {
    std::vector<SingleElement<std::string>> mothers_;
    std::vector<SingleElement<std::string>> disjoints_;
    std::map<std::string, std::vector<std::string>> dictionary_;
    std::map<std::string, std::vector<std::string>> muted_dictionary_;

    std::vector<PairElement<std::string, std::string>> object_relations_;
    std::vector<PairElement<std::string, std::string>> data_relations_;
    std::vector<std::string> equivalences_;
  };

  // for friend
  class ClassDrawer;
  class ObjectPropertyGraph;
  class DataPropertyGraph;
  class IndividualGraph;
  class ClassChecker;
  class AnonymousClassGraph;

  class ClassGraph : public OntoGraph<ClassBranch>
  {
    friend ClassDrawer;
    friend ObjectPropertyGraph;
    friend DataPropertyGraph;
    friend IndividualGraph;
    friend AnonymousClassGraph;

    friend ClassChecker;

  public:
    ClassGraph(IndividualGraph* individual_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
    ClassGraph(const ClassGraph& other, IndividualGraph* individual_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
    ~ClassGraph() override = default;

    ClassBranch* add(const std::string& value, ObjectVectors_t& object_vector);
    void add(std::vector<std::string>& disjoints);

    void deepCopy(const ClassGraph& other);

    std::unordered_set<std::string> getRelationFrom(const std::string& class_name, int depth = -1); // C3
    std::unordered_set<index_t> getRelationFrom(index_t class_id, int depth = -1);
    std::unordered_set<std::string> getRelatedFrom(const std::string& property); // C3
    std::unordered_set<index_t> getRelatedFrom(index_t property);
    std::unordered_set<std::string> getRelationOn(const std::string& class_name, int depth = -1); // C4
    std::unordered_set<index_t> getRelationOn(index_t class_id, int depth);
    std::unordered_set<std::string> getRelatedOn(const std::string& property); // C3
    std::unordered_set<index_t> getRelatedOn(index_t property);
    std::unordered_set<std::string> getRelationWith(const std::string& class_name); // C3
    std::unordered_set<index_t> getRelationWith(index_t class_id);
    std::unordered_set<std::string> getRelatedWith(const std::string& class_name); // C3
    std::unordered_set<index_t> getRelatedWith(index_t class_id);
    std::unordered_set<std::string> getFrom(const std::string& param);
    std::unordered_set<std::string> getFrom(const std::string& class_name, const std::string& property);
    std::unordered_set<index_t> getFrom(index_t class_id, index_t property);
    std::unordered_set<std::string> getOn(const std::string& param);
    std::unordered_set<std::string> getOn(const std::string& class_name, const std::string& property);
    std::unordered_set<index_t> getOn(index_t class_id, index_t property);
    std::unordered_set<std::string> getWith(const std::string& param, int depth = -1);
    std::unordered_set<std::string> getWith(const std::string& first_class, const std::string& second_class, int depth = -1);
    std::unordered_set<index_t> getWith(index_t first_class, index_t second_class, int depth = -1);
    std::unordered_set<std::string> getDomainOf(const std::string& class_name, int depth = -1);
    std::unordered_set<index_t> getDomainOf(index_t class_id, int depth = -1);
    std::unordered_set<std::string> getRangeOf(const std::string& class_name, int depth = -1);
    std::unordered_set<index_t> getRangeOf(index_t class_id, int depth = -1);

    void getDownIndividual(ClassBranch* branch, std::unordered_set<std::string>& res, bool single_same = false);
    void getDownIndividual(ClassBranch* branch, std::unordered_set<index_t>& res, bool single_same = false);
    std::unordered_set<IndividualBranch*> getDownIndividualPtrSafe(ClassBranch* branch, size_t depth = -1);
    void getDownIndividualPtr(ClassBranch* branch, std::unordered_set<IndividualBranch*>& res, size_t depth = -1, size_t current_depth = 0);

    void deleteClass(ClassBranch* class_branch);
    bool addInheritage(const std::string& branch_base, const std::string& branch_inherited);
    int deleteRelationsOnClass(ClassBranch* class_branch, std::vector<ClassBranch*> vect);
    void addRelation(ClassBranch*, const std::string& property, const std::string& class_on);
    void addRelation(ClassBranch*, const std::string& property, const std::string& type, const std::string& data);
    void addRelationInvert(const std::string& class_from, const std::string& property, ClassBranch* class_on);
    void removeRelation(const std::string& class_from, const std::string& property, const std::string& class_on);
    void removeRelation(const std::string& class_from, const std::string& property, const std::string& type, const std::string& data);

    std::pair<bool, ClassBranch*> checkDomainOrRange(const std::unordered_set<ClassBranch*>& domain_or_range, const std::unordered_set<ClassBranch*>& classes);

  private:
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;

    void addObjectRelation(ClassBranch* me, PairElement<std::string, std::string>& relation);
    void addDataRelation(ClassBranch* me, PairElement<std::string, std::string>& relation);

    template<typename T>
    std::unordered_set<T> getRelationFrom(ClassBranch* class_branch, int depth = -1);
    template<typename T>
    void getRelationFrom(ClassBranch* class_branch, std::unordered_set<T>& res, int depth);
    template<typename T>
    void getRelatedFrom(const std::unordered_set<index_t>& object_properties, const std::unordered_set<index_t>& data_properties, std::unordered_set<T>& res);
    template<typename T>
    void getRelationOnObjectProperties(const std::string& class_name, std::unordered_set<T>& res, int depth);
    void getRelationOnDataProperties(const std::string& class_name, std::unordered_set<std::string>& res, int depth);
    void getRelationOnDataProperties(const std::string& class_name, std::unordered_set<index_t>& res, int depth);
    void getRelatedOnDataProperties(const std::string& property, std::unordered_set<std::string>& res);
    void getRelatedOnDataProperties(index_t property, std::unordered_set<index_t>& res);
    void getRelationWith(ClassBranch* class_branch, std::map<index_t, int>& properties, std::vector<int>& depths, std::vector<std::string>& res, int depth);
    void getRelationWith(ClassBranch* class_branch, std::map<index_t, int>& properties, std::vector<int>& depths, std::vector<index_t>& res, int depth);
    template<typename T>
    void dataGetRelatedWith(ClassBranch* class_branch, index_t property, LiteralNode* data, std::unordered_set<T>& res, std::unordered_set<index_t>& do_not_take);
    template<typename T>
    void objectGetRelatedWith(ClassBranch* class_branch, index_t property, index_t class_id, std::unordered_set<T>& res, std::unordered_set<index_t>& do_not_take);
    template<typename T>
    void getOn(ClassBranch* class_branch, std::unordered_set<index_t>& object_properties, std::unordered_set<index_t>& data_properties, std::unordered_set<T>& res, uint32_t current_depth, int& found_depth);
    void getWith(ClassBranch* first_class, index_t second_class, std::unordered_set<std::string>& res, std::unordered_set<index_t>& do_not_take, uint32_t current_depth, int& found_depth, int depth_prop, std::unordered_set<ClassBranch*>& next_step);
    void getWith(ClassBranch* first_class, index_t second_class, std::unordered_set<index_t>& res, std::unordered_set<index_t>& do_not_take, uint32_t current_depth, int& found_depth, int depth_prop, std::unordered_set<ClassBranch*>& next_step);
    template<typename T>
    void getWith_(ClassBranch* first_class, index_t second_class, std::unordered_set<T>& res, std::unordered_set<index_t>& do_not_take, uint32_t current_depth, int& found_depth, int depth_prop, std::unordered_set<ClassBranch*>& next_step);
    void getDomainOf(ClassBranch* branch, std::unordered_set<std::string>& res, int depth = -1);
    void getDomainOf(ClassBranch* branch, std::unordered_set<index_t>& res, int depth = -1);
    void getRangeOf(ClassBranch* branch, std::unordered_set<std::string>& res, int depth = -1);
    void getRangeOf(ClassBranch* branch, std::unordered_set<index_t>& res, int depth = -1);

    bool checkRangeAndDomain(ClassBranch* from, ObjectPropertyBranch* prop, ClassBranch* on);
    bool checkRangeAndDomain(ClassBranch* from, DataPropertyBranch* prop, LiteralNode* data);

    void cpyBranch(ClassBranch* old_branch, ClassBranch* new_branch);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSGRAPH_H
