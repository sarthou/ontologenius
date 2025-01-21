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

  private:
    RuleGraph* rule_graph_;

    void writeRule(RuleBranch* branch);
    void writeVariable(const std::string& rule_variable);

    void writeAtom(const std::vector<RuleTriplet_t>& atom_list, const RuleTriplet_t& current_atom, size_t level, size_t index = 0);
    void writeClassAtom(const RuleTriplet_t& class_atom, size_t level);
    void writeObjectAtom(const RuleTriplet_t& object_atom, size_t level);
    void writeDataAtom(const RuleTriplet_t& data_atom, size_t level);
    void writeBuiltinAtom(const RuleTriplet_t& builtin_atom, size_t level);

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
  };
} // namespace ontologenius

#endif // ONTOLOGENIUS_RULEOWLWRITER_H