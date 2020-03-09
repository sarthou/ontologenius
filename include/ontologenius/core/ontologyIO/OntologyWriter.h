#ifndef ONTOLOGENIUS_ONTOLOGYWRITER_H
#define ONTOLOGENIUS_ONTOLOGYWRITER_H

#include <string>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

class Ontology;

class OntologyWriter
{
public:
  OntologyWriter(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
  OntologyWriter(Ontology& onto);
  ~OntologyWriter() {}

  void setFileName(const std::string& name) {file_name_ = name; }
  std::string getFileName() { return file_name_; }
  void write(const std::string& file_name = "none");

private:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;
  IndividualGraph* individual_graph_;

  std::string file_name_;
  FILE* file_;

  void writeStart();
  void writeEnd();
  void writeBanner(const std::string& name);
  void writeString(const std::string& text);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYWRITER_H
