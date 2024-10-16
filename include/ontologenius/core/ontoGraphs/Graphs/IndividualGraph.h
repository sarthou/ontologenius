#ifndef ONTOLOGENIUS_INDIVIDUALGRAPH_H
#define ONTOLOGENIUS_INDIVIDUALGRAPH_H

#include <cstdint>
#include <map>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/Graph.h"

namespace ontologenius {

  struct IndividualVectors_t
  {
    std::vector<SingleElement<std::string>> is_a_;

    std::vector<PairElement<std::string, std::string>> object_relations_;
    std::vector<PairElement<std::string, std::string>> data_relations_;

    std::vector<SingleElement<std::string>> same_as_;
    // TODO : add vector distinct
    std::map<std::string, std::vector<std::string>> dictionary_;
    std::map<std::string, std::vector<std::string>> muted_dictionary_;
  };

  // for friend
  class IndividualChecker;
  class AnonymousGraph;

  // for graphs usage
  class ClassGraph;
  class ObjectPropertyGraph;
  class DataPropertyGraph;

  class IndividualGraph : public Graph<IndividualBranch>
  {
    friend IndividualChecker;
    friend AnonymousGraph;

  public:
    IndividualGraph(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
    IndividualGraph(const IndividualGraph& other, ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph);
    ~IndividualGraph() override = default;

    void deepCopy(const IndividualGraph& other);

    IndividualBranch* add(const std::string& value, IndividualVectors_t& individual_vector);
    void add(std::vector<std::string>& distinct_);

    std::unordered_set<std::string> getSame(const std::string& individual); // C1
    std::unordered_set<index_t> getSame(index_t individual);
    std::unordered_set<std::string> getDistincts(const std::string& individual); // C2
    std::unordered_set<index_t> getDistincts(index_t individual);
    std::unordered_set<std::string> getRelationFrom(const std::string& individual, int depth = -1); // C3
    std::unordered_set<index_t> getRelationFrom(index_t individual, int depth);
    std::unordered_set<std::string> getRelatedFrom(const std::string& property); // C3
    std::unordered_set<index_t> getRelatedFrom(index_t property);
    std::unordered_set<std::string> getRelationOn(const std::string& individual, int depth = -1); // C4
    std::unordered_set<index_t> getRelationOn(index_t individual, int depth = -1);
    std::unordered_set<std::string> getRelatedOn(const std::string& property); // C3
    std::unordered_set<index_t> getRelatedOn(index_t property);
    std::unordered_set<std::string> getRelationWith(const std::string& individual); // C3
    std::unordered_set<index_t> getRelationWith(index_t individual);
    std::unordered_set<std::string> getRelatedWith(const std::string& individual); // C3
    std::unordered_set<index_t> getRelatedWith(index_t individual);
    std::unordered_set<std::string> getFrom(const std::string& param);
    std::unordered_set<std::string> getFrom(const std::string& individual, const std::string& property, bool single_same = false);
    std::unordered_set<index_t> getFrom(index_t individual, index_t property, bool single_same = false);
    std::unordered_set<std::string> getOn(const std::string& param);
    std::unordered_set<std::string> getOn(const std::string& individual, const std::string& property, bool single_same = false);
    std::unordered_set<index_t> getOn(index_t individual, index_t property, bool single_same = false);
    std::unordered_set<std::string> getWith(const std::string& param, int depth = -1);
    std::unordered_set<std::string> getWith(const std::string& first_individual, const std::string& second_individual, int depth = -1);
    std::unordered_set<index_t> getWith(index_t first_individual, index_t second_individual, int depth = -1);
    std::unordered_set<std::string> getDomainOf(const std::string& individual, int depth = -1);
    std::unordered_set<index_t> getDomainOf(index_t individual, int depth = -1);
    std::unordered_set<std::string> getRangeOf(const std::string& individual, int depth = -1);
    std::unordered_set<index_t> getRangeOf(index_t individual, int depth = -1);
    std::unordered_set<std::string> getUp(const std::string& individual, int depth = -1); // C3
    std::unordered_set<index_t> getUp(index_t individual, int depth = -1);
    std::unordered_set<std::string> select(const std::unordered_set<std::string>& on, const std::string& class_selector);
    std::unordered_set<index_t> select(const std::unordered_set<index_t>& on, index_t class_selector);

    std::unordered_set<std::string> getType(const std::string& class_selector, bool single_same = false);
    std::unordered_set<index_t> getType(index_t class_selector, bool single_same = false);
    bool isA(const std::string& indiv, const std::string& class_selector);
    bool isA(index_t indiv, index_t class_selector);
    bool isA(IndividualBranch* indiv, const std::string& class_selector);
    bool isA(IndividualBranch* indiv, index_t class_selector);
    bool relationExists(const std::string& param);
    bool relationExists(const std::string& subject, const std::string& property, const std::string& object);

    ClassBranch* upgradeToBranch(IndividualBranch* indiv);
    IndividualBranch* findOrCreateBranchSafe(const std::string& name);
    IndividualBranch* findOrCreateBranch(const std::string& name);
    void deleteIndividual(IndividualBranch* indiv);
    void redirectDeleteIndividual(IndividualBranch* indiv, ClassBranch* class_branch);
    bool addInheritage(const std::string& indiv, const std::string& class_inherited);
    bool addInheritage(IndividualBranch* branch, const std::string& class_inherited);
    bool addInheritageUnsafe(IndividualBranch* branch, const std::string& class_inherited);
    bool addInheritageInvert(const std::string& indiv, const std::string& class_inherited);
    bool addInheritageInvertUpgrade(const std::string& indiv, const std::string& class_inherited);
    int addRelation(IndividualBranch* indiv_from, ObjectPropertyBranch* property, IndividualBranch* indiv_on, double proba = 1.0, bool infered = false, bool check_existance = true);
    int addRelation(IndividualBranch* indiv_from, DataPropertyBranch* property, LiteralNode* data, double proba = 1.0, bool infered = false);
    void addRelation(IndividualBranch* indiv_from, const std::string& property, const std::string& indiv_on);
    void addRelation(IndividualBranch* indiv_from, const std::string& property, const std::string& type, const std::string& data);
    void addRelationInvert(const std::string& indiv_from, const std::string& property, IndividualBranch* indiv_on);
    std::vector<std::pair<std::string, std::string>> removeInheritage(const std::string& indiv, const std::string& class_inherited);
    bool removeInheritage(IndividualBranch* indiv, ClassBranch* class_branch, std::vector<std::pair<std::string, std::string>>& explanations, bool protect_stated = false);
    void addSameAs(const std::string& indiv_1, const std::string& indiv_2);
    std::vector<std::pair<std::string, std::string>> removeSameAs(const std::string& indiv_1, const std::string& indiv_2, bool protect_stated = false);
    std::pair<std::vector<std::pair<std::string, std::string>>, bool> removeRelation(IndividualBranch* branch_from, ObjectPropertyBranch* property, IndividualBranch* branch_on, bool protect_stated = false);
    std::vector<std::pair<std::string, std::string>> removeRelation(const std::string& indiv_from, const std::string& property, const std::string& indiv_on);
    std::vector<std::pair<std::string, std::string>> removeRelation(const std::string& indiv_from, const std::string& property, const std::string& type, const std::string& data);
    std::vector<std::pair<std::string, std::string>> removeRelationInverse(IndividualBranch* indiv_from, ObjectPropertyBranch* property, IndividualBranch* indiv_on);
    std::vector<std::pair<std::string, std::string>> removeRelationSymetric(IndividualBranch* indiv_from, ObjectPropertyBranch* property, IndividualBranch* indiv_on);
    template<typename T, typename C>
    std::vector<std::pair<std::string, std::string>> removeInductions(IndividualBranch* indiv_from, RelationsWithInductions<PairElement<T, C>>& relations, size_t relation_index);
    template<typename T>
    std::vector<std::pair<std::string, std::string>> removeInductions(IndividualBranch* indiv_from, RelationsWithInductions<SingleElement<T>>& relations, size_t relation_index, const std::string& property);

    void getUpPtr(IndividualBranch* indiv, std::unordered_set<ClassBranch*>& res, int depth = -1, uint32_t current_depth = 0);
    void getLowestSame(IndividualBranch* individual, std::unordered_set<std::string>& res);
    void getSame(IndividualBranch* individual, std::unordered_set<std::string>& res);
    void getLowestSame(IndividualBranch* individual, std::unordered_set<index_t>& res);
    void getSame(IndividualBranch* individual, std::unordered_set<index_t>& res);

  private:
    ClassGraph* class_graph_;
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;

    std::vector<IndividualBranch*> ordered_individuals_; // contains the individuals ordered wrt their index
                                                         // unused indexes have nullptr in

    template<typename T>
    std::unordered_set<T> getDistincts(IndividualBranch* individual);
    template<typename T>
    std::unordered_set<T> getRelationFrom(IndividualBranch* individual, int depth);
    template<typename T>
    std::unordered_set<T> getRelatedFrom(const T& property);
    template<typename T>
    void getRelatedOn(const T& property, std::unordered_set<T>& res);
    template<typename T>
    void getUp(IndividualBranch* indiv, std::unordered_set<T>& res, int depth = -1, uint32_t current_depth = 0);
    template<typename T>
    void getRelatedWith(index_t individual, std::unordered_set<T>& res);
    template<typename T>
    void getFrom(index_t individual, const T& property, std::unordered_set<T>& res, bool single_same = false);
    template<typename T>
    std::unordered_set<T> getOn(IndividualBranch* individual, const T& property, bool single_same = false);
    template<typename T>
    void getWith(IndividualBranch* first_individual, const std::unordered_set<index_t>& second_individual_index, std::unordered_set<T>& res, int depth);
    template<typename T>
    void getDomainOf(IndividualBranch* individual, std::unordered_set<T>& res, int depth);
    template<typename T>
    void getRangeOf(IndividualBranch* individual, std::unordered_set<T>& res, int depth);
    template<typename T>
    bool isATemplate(IndividualBranch* branch, const T& class_selector);

    void addSames(IndividualBranch* me, const std::vector<SingleElement<std::string>>& sames, bool is_new = true);
    void addObjectRelation(IndividualBranch* me, PairElement<std::string, std::string>& relation);
    void addDataRelation(IndividualBranch* me, PairElement<std::string, std::string>& relation);

    template<typename T>
    void getRelationFrom(ClassBranch* class_branch, std::unordered_set<T>& res, int depth = -1);
    bool getRelatedWith(ClassBranch* class_branch, index_t data, std::unordered_set<ClassBranch*>& next_step, std::unordered_set<index_t>& took);
    bool getFrom(ClassBranch* class_branch, const std::unordered_set<index_t>& object_properties, const std::unordered_set<index_t>& data_properties, index_t data, const std::unordered_set<index_t>& down_classes, std::unordered_set<ClassBranch*>& next_step, std::unordered_set<index_t>& do_not_take);

    bool relationExists(IndividualBranch* subject, ObjectPropertyBranch* property, IndividualBranch* object);

    void getDistincts(IndividualBranch* individual, std::unordered_set<IndividualBranch*>& res);
    std::unordered_set<index_t> getSameId(const std::string& individual);
    std::unordered_set<index_t> getSameId(index_t individual);
    void getLowestSame(IndividualBranch* individual, std::unordered_set<IndividualBranch*>& res);
    void getSame(IndividualBranch* individual, std::unordered_set<IndividualBranch*>& res);
    void getSame(IndividualBranch* individual, std::vector<IndividualBranch*>& res);
    std::unordered_set<std::string> getSame(IndividualBranch* individual);
    std::unordered_set<index_t> getSameId(IndividualBranch* individual);

    bool checkRangeAndDomain(IndividualBranch* from, ObjectPropertyBranch* prop, IndividualBranch* on);
    bool checkRangeAndDomain(IndividualBranch* from, DataPropertyBranch* prop, LiteralNode* data);

    void cpyBranch(IndividualBranch* old_branch, IndividualBranch* new_branch);
    void insertBranchInVectors(IndividualBranch* branch);
    void removeBranchInVectors(size_t vector_index);
  };

  template<typename T, typename C>
  std::vector<std::pair<std::string, std::string>> IndividualGraph::removeInductions(IndividualBranch* indiv_from, RelationsWithInductions<PairElement<T, C>>& relations, size_t relation_index)
  {
    std::vector<std::pair<std::string, std::string>> explanations;

    auto relation = relations[relation_index];
    auto property = relation.first;
    auto indiv_on = relation.second;

    for(size_t i = 0; i < relations.has_induced_object_relations[relation_index]->triplets.size();)
    {
      auto triplet = relations.has_induced_object_relations[relation_index]->triplets[i];
      auto tmp = removeRelation(triplet.subject,
                                triplet.predicate,
                                triplet.object,
                                true);
      if(tmp.second)
      {
        explanations.emplace_back("[DEL]" + triplet.subject->value() + "|" +
                                    triplet.predicate->value() + "|" +
                                    triplet.object->value(),
                                  "[DEL]" + indiv_from->value() + "|" + property->value() + "|" + indiv_on->value());
        explanations.insert(explanations.end(), tmp.first.begin(), tmp.first.end());
      }
      else
        i++; // we enter in this case if the induced relation has later been stated and can thus not be removed automatically
    }

    for(size_t i = 0; i < relations.has_induced_inheritance_relations[relation_index]->triplets.size();)
    {
      auto triplet = relations.has_induced_inheritance_relations[relation_index]->triplets[i];

      std::vector<std::pair<std::string, std::string>> tmp;
      if(removeInheritage(triplet.subject, triplet.object, tmp, true))
      {
        explanations.emplace_back("[DEL]" + triplet.subject->value() + "|isA|" + triplet.object->value(),
                                  "[DEL]" + indiv_from->value() + "|" + property->value() + "|" + indiv_on->value());
        explanations.insert(explanations.end(), tmp.begin(), tmp.end());
      }
      else
        i++; // we enter in this case if the induced relation has later been stated and can thus not be removed automatically
    }

    return explanations;
  }

  template<typename T>
  std::vector<std::pair<std::string, std::string>> IndividualGraph::removeInductions(IndividualBranch* indiv_from, RelationsWithInductions<SingleElement<T>>& relations, size_t relation_index, const std::string& property)
  {
    std::vector<std::pair<std::string, std::string>> explanations;

    auto relation = relations[relation_index];
    auto* indiv_on = relation.elem;

    for(size_t i = 0; i < relations.has_induced_object_relations[relation_index]->triplets.size();)
    {
      auto triplet = relations.has_induced_object_relations[relation_index]->triplets[i];
      auto tmp = removeRelation(triplet.subject,
                                triplet.predicate,
                                triplet.object,
                                true);
      if(tmp.second)
      {
        explanations.emplace_back("[DEL]" + triplet.subject->value() + "|" +
                                    triplet.predicate->value() + "|" +
                                    triplet.object->value(),
                                  "[DEL]" + indiv_from->value() + "|" + property + "|" + indiv_on->value());
        explanations.insert(explanations.end(), tmp.first.begin(), tmp.first.end());
      }
      else
        i++; // we enter in this case if the induced relation has later been stated and can thus not be removed automatically
    }

    for(size_t i = 0; i < relations.has_induced_inheritance_relations[relation_index]->triplets.size();)
    {
      auto triplet = relations.has_induced_inheritance_relations[relation_index]->triplets[i];

      std::vector<std::pair<std::string, std::string>> tmp;
      if(removeInheritage(triplet.subject, triplet.object, tmp, true))
      {
        explanations.emplace_back("[DEL]" + triplet.subject->value() + "|isA|" +
                                    triplet.object->value(),
                                  "[DEL]" + indiv_from->value() + "|" + property + "|" + indiv_on->value());
        explanations.insert(explanations.end(), tmp.begin(), tmp.end());
      }
      else
        i++; // we enter in this case if the induced relation has later been stated and can thus not be removed automatically
    }

    return explanations;
  }

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALGRAPH_H
