#ifndef CLASSWRITER_H
#define CLASSWRITER_H

#include <string>

class ClassGraph;
class ClassBranch_t;

class ClassWriter
{
public:
  ClassWriter(ClassGraph* class_graph) {class_graph_ = class_graph; file_ = nullptr; };
  ~ClassWriter() {};

  void write(FILE* file);

private:
  ClassGraph* class_graph_;
  FILE* file_;

  void writeClass(ClassBranch_t* branch);

  void writeString(std::string text);
};

#endif
