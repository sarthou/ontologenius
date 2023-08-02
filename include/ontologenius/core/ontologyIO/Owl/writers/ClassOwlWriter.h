#ifndef ONTOLOGENIUS_CLASSOWLWRITER_H
#define ONTOLOGENIUS_CLASSOWLWRITER_H

#include <string>
#include <vector>
#include <set>

#include "ontologenius/core/ontologyIO/Owl/writers/NodeOwlWriter.h"

namespace ontologenius {

class ClassGraph;
class ClassBranch_t;

class AnonymousClassGraph;
class AnonymousClassBranch_t;
class AnonymousClassElement_t;

class ClassOwlWriter : private NodeOwlWriter
{
public:
  ClassOwlWriter(ClassGraph* class_graph, AnonymousClassGraph* anonymous_graph, const std::string& ns);
  ~ClassOwlWriter() {};

  void write(FILE* file);
  void writeGeneralAxioms(FILE* file);

private:
  ClassGraph* class_graph_;
  AnonymousClassGraph* anonymous_graph_;

  void writeClass(ClassBranch_t* branch);
  void writeSubClassOf(ClassBranch_t* branch);
  void writeEquivalentClass(ClassBranch_t* branch);
  void writeCollection(AnonymousClassElement_t* ano_elem, int nb_ident);
  void writeRestriction(AnonymousClassElement_t* ano_elem, int nb_indent);
  std::string writeCardinality(AnonymousClassElement_t* ano_element);
  void writeDisjointWith(ClassBranch_t* branch);
  void writeDisjointWith(std::vector<ClassBranch_t*>& classes);
  void getDisjointsSets(ClassBranch_t* base, std::set<std::set<ClassBranch_t*>>& res);
  void getDisjointsSets(ClassBranch_t* last, const std::set<ClassBranch_t*>& base_set, const std::set<ClassBranch_t*>& restriction_set, std::set<std::set<ClassBranch_t*>>& res);
  void writeObjectProperties(ClassBranch_t* branch);
  void writeDataProperties(ClassBranch_t* branch);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSOWLWRITER_H
