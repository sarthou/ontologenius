#ifndef ONTOLOGENIUS_CLASSWRITER_H
#define ONTOLOGENIUS_CLASSWRITER_H

#include <string>
#include <vector>
#include <set>

#include "ontologenius/core/ontologyIO/writers/NodeWriter.h"

namespace ontologenius {

class ClassGraph;
class ClassBranch_t;

class ClassWriter : private NodeWriter
{
public:
  ClassWriter(ClassGraph* class_graph, const std::string& ns);
  ~ClassWriter() {};

  void write(FILE* file);
  void writeGeneralAxioms(FILE* file);

private:
  ClassGraph* class_graph_;

  void writeClass(ClassBranch_t* branch);
  void writeSubClassOf(ClassBranch_t* branch);
  void writeDisjointWith(ClassBranch_t* branch);
  void writeDisjointWith(std::vector<ClassBranch_t*>& classes);
  void getDisjointsSets(ClassBranch_t* base, std::set<std::set<ClassBranch_t*>>& res);
  void getDisjointsSets(ClassBranch_t* last, const std::set<ClassBranch_t*>& base_set, const std::set<ClassBranch_t*>& restriction_set, std::set<std::set<ClassBranch_t*>>& res);
  void writeObjectProperties(ClassBranch_t* branch);
  void writeDataProperties(ClassBranch_t* branch);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSWRITER_H
