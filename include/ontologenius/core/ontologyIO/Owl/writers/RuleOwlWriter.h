#ifndef ONTOLOGENIUS_RULEOWLWRITER_H
#define ONTOLOGENIUS_RULEOWLWRITER_H

#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"
#include "ontologenius/core/ontologyIO/Owl/writers/NodeOwlWriter.h"

namespace ontologenius {

  class RuleGraph;
  class RuleBranch;
  class AnonymousClassElement;

  class RuleOwlWriter : private NodeOwlWriter
  {
  public:
    RuleOwlWriter(RuleGraph* rule_graph, const std::string& ns);
    ~RuleOwlWriter() = default;

    void write(FILE* file);
    void writeGeneralAxioms(FILE* file);

  private:
    RuleGraph* rule_graph_;

    void writeRule(RuleBranch* branch);
    void writeVariable(Variable_t rule_variable);
    void writeRuleImpl(RuleBranch* branch);
    void writeClassPredicate(ClassAtom* class_atom);
    void writeObjectPropertyPredicate(ObjectPropertyAtom* object_atom);
    void writeDataPropertyPredicate(DataPropertyAtom* data_atom);

    // Anonymous classes method duplicates
    void writeEquivalentClass(ClassBranch* branch);
    void writeRestriction(AnonymousClassElement* ano_elem, size_t level);
    void writeClassExpression(AnonymousClassElement* ano_elem, size_t level);
    void writeDatatypeExpression(AnonymousClassElement* ano_elem, size_t level);
    void writeIntersection(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop = false);
    void writeUnion(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop = false);
    void writeOneOf(AnonymousClassElement* ano_elem, size_t level);
    void writeComplement(AnonymousClassElement* ano_elem, size_t level);
    void writeDataComplement(AnonymousClassElement* ano_elem, size_t level);
    void writeComplexDescription(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop = false);
    void writeCardinalityValue(AnonymousClassElement* ano_elem, size_t level);
    void writeCardinalityRange(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop);
    void writeCardinality(AnonymousClassElement* ano_element, size_t level);

    std::string getResource(AnonymousClassElement* ano_elem, const std::string& attribute_name = "rdf:resource", bool used_property = false);

    void writeDisjointWith(ClassBranch* branch);
    void writeDisjointWith(std::vector<ClassBranch*>& classes);
    void getDisjointsSets(ClassBranch* base, std::set<std::set<ClassBranch*>>& res);
    void getDisjointsSets(ClassBranch* last, const std::set<ClassBranch*>& base_set, const std::set<ClassBranch*>& restriction_set, std::set<std::set<ClassBranch*>>& res);
    void writeObjectProperties(ClassBranch* branch);
    void writeDataProperties(ClassBranch* branch);
  };
} // namespace ontologenius

#endif // ONTOLOGENIUS_RULEOWLWRITER_H