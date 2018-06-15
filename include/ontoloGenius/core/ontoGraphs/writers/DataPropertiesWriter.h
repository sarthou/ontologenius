#ifndef DATAPROPERTIESWRITER_H
#define DATAPROPERTIESWRITER_H

#include "ontoloGenius/core/ontoGraphs/writers/NodeWriter.h"

#include <string>

class DataPropertyGraph;
class DataPropertyBranch_t;

class DataPropertiesWriter : public NodeWriter
{
public:
  DataPropertiesWriter(DataPropertyGraph* property_graph) {property_graph_ = property_graph; };
  ~DataPropertiesWriter() {};

  void write(FILE* file);

private:
  DataPropertyGraph* property_graph_;

  void writeProperty(DataPropertyBranch_t* branch);
  void writeSubPropertyOf(DataPropertyBranch_t* branch);
};

#endif
