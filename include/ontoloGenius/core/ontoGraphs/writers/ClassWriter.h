#ifndef CLASSWRITER_H
#define CLASSWRITER_H

#include "ontoloGenius/core/ontoGraphs/writers/NodeWriter.h"

#include <string>

class ClassGraph;
class ClassBranch_t;

class ClassWriter : private NodeWriter
{
public:
  ClassWriter(ClassGraph* class_graph) {class_graph_ = class_graph; };
  ~ClassWriter() {};

  void write(FILE* file);

private:
  ClassGraph* class_graph_;

  void writeClass(ClassBranch_t* branch);
  void writeSubClassOf(ClassBranch_t* branch);
  void writeDisjointWith(ClassBranch_t* branch);
};

#endif
