#ifndef OBJECTPROPERTIESWRITER_H
#define OBJECTPROPERTIESWRITER_H

#include "ontoloGenius/core/ontologyIO/writers/PropertiesWriter.h"

#include <string>

class ObjectPropertyGraph;
class ObjectPropertyBranch_t;

class ObjectPropertiesWriter : public PropertiesWriter<ObjectPropertyBranch_t>
{
public:
  ObjectPropertiesWriter(ObjectPropertyGraph* property_graph) {property_graph_ = property_graph; };
  ~ObjectPropertiesWriter() {};

  void write(FILE* file);

private:
  ObjectPropertyGraph* property_graph_;

  void writeProperty(ObjectPropertyBranch_t* branch);
  void writeSubPropertyOf(ObjectPropertyBranch_t* branch);
  void writeInverseOf(ObjectPropertyBranch_t* branch);
  void writeRange(ObjectPropertyBranch_t* branch);
  void writeDomain(ObjectPropertyBranch_t* branch);
  void writeChain(ObjectPropertyBranch_t* branch);
};

#endif
