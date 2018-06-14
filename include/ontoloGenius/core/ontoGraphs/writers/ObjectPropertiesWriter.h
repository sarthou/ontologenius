#ifndef OBJECTPROPERTIESWRITER_H
#define OBJECTPROPERTIESWRITER_H

#include <string>

class ObjectPropertyGraph;
class ObjectPropertyBranch_t;

class ObjectPropertiesWriter
{
public:
  ObjectPropertiesWriter(ObjectPropertyGraph* property_graph) {property_graph_ = property_graph; file_ = nullptr; };
  ~ObjectPropertiesWriter() {};

  void write(FILE* file);

private:
  ObjectPropertyGraph* property_graph_;
  FILE* file_;

  void writeProperty(ObjectPropertyBranch_t* branch);

  void writeString(std::string text);
};

#endif
