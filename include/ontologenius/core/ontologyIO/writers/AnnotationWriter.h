#ifndef ONTOLOGENIUS_ANNOTATIONWRITER_H
#define ONTOLOGENIUS_ANNOTATIONWRITER_H

#include <string>
#include <vector>

#include "ontologenius/core/ontologyIO/writers/NodeWriter.h"

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

class AnnotationWriter : private NodeWriter
{
public:
  AnnotationWriter(ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, const std::string& ns);

  void write(FILE* file);

private:
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;

  void writeAnnotation(ObjectPropertyBranch_t* branch);
  void writeAnnotation(DataPropertyBranch_t* branch);
  void writeSubPropertyOf(ObjectPropertyBranch_t* branch);
  void writeSubPropertyOf(DataPropertyBranch_t* branch);
  void writeRange(const std::vector<data_t>& ranges);
  void writeRange(const std::vector<ClassElement_t>& ranges);
  void writeDomain(const std::vector<ClassElement_t>& domains);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANNOTATIONWRITER_H
