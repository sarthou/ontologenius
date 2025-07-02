#ifndef ONTOLOGENIUS_CLASSOWLWRITER_H
#define ONTOLOGENIUS_CLASSOWLWRITER_H

#include <cstddef>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include "ontologenius/core/ontologyIO/Owl/writers/NodeOwlWriter.h"

namespace ontologenius {

  class ClassGraph;
  class ClassBranch;
  class AnonymousClassElement;

  class ClassOwlWriter : private NodeOwlWriter
  {
  public:
    ClassOwlWriter(ClassGraph* class_graph, const std::string& ns);
    ~ClassOwlWriter() = default;

    void write(FILE* file);
    void writeGeneralAxioms(FILE* file);

  private:
    ClassGraph* class_graph_;

    void writeClass(ClassBranch* branch);
    void writeSubClassOf(ClassBranch* branch);

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

#endif // ONTOLOGENIUS_CLASSOWLWRITER_H
