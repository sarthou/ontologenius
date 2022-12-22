#ifndef ONTOLOGENIUS_ONTOLOGYLOADER_H
#define ONTOLOGENIUS_ONTOLOGYLOADER_H

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include "ontologenius/core/ontologyIO/Owl/OntologyOwlReader.h"
#include "ontologenius/core/ontologyIO/Turtle/OntologyTtlReader.h"

namespace ontologenius {

class Ontology;

class OntologyLoader
{
public:
  OntologyLoader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
  explicit OntologyLoader(Ontology& onto);
  ~OntologyLoader() {}

  int loadFile(const std::string& file);
  int loadUri(const std::string& uri);

  int loadIndividuals();

  int getNbLoadedElements();
  bool isEmpty() { return getNbLoadedElements() == 0; }

  void setDisplay(bool display);

private:
  OntologyOwlReader owl_reader_;
  OntologyTtlReader ttl_reader_;

  std::vector<std::string> files_;
  std::vector<std::string> uri_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYLOADER_H