#ifndef ONTOLOGENIUS_ONTOLOGYREADER_H
#define ONTOLOGENIUS_ONTOLOGYREADER_H

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

class Ontology;

class OntologyReader
{
public:
  OntologyReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
  OntologyReader(Ontology& onto);
  ~OntologyReader() {}

  void setDisplay(bool display) { display_ = display; }
  bool empty() {return (elemLoaded == 0); }

protected:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;
  IndividualGraph* individual_graph_;

  int elemLoaded;
  bool display_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYREADER_H