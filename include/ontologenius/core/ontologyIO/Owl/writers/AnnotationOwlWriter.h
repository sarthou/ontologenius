#ifndef ONTOLOGENIUS_ANNOTATIONOWLWRITER_H
#define ONTOLOGENIUS_ANNOTATIONOWLWRITER_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontologyIO/Owl/writers/NodeOwlWriter.h"

namespace ontologenius {

  class AnnotationOwlWriter : private NodeOwlWriter
  {
  public:
    AnnotationOwlWriter(ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, const std::string& ns);

    void write(FILE* file);

  private:
    ObjectPropertyGraph* object_property_graph_;
    DataPropertyGraph* data_property_graph_;

    void writeAnnotation(ObjectPropertyBranch_t* branch);
    void writeAnnotation(DataPropertyBranch_t* branch);
    void writeSubPropertyOf(ObjectPropertyBranch_t* branch);
    void writeSubPropertyOf(DataPropertyBranch_t* branch);
    void writeRange(const std::vector<LiteralNode*>& ranges);
    void writeRange(const std::vector<ClassElement_t>& ranges);
    void writeDomain(const std::vector<ClassElement_t>& domains);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_ANNOTATIONOWLWRITER_H
