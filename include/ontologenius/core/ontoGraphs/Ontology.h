#ifndef ONTOLOGENIUS_ONTOLOGY_H
#define ONTOLOGENIUS_ONTOLOGY_H

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"

#include "ontologenius/core/ontologyIO/OntologyLoader.h"
#include "ontologenius/core/ontologyIO/Owl/OntologyOwlWriter.h"

namespace ontologenius {

class Ontology
{
public:
  Ontology(const std::string& language = "en");
  Ontology(const Ontology& other);
  ~Ontology();

  bool close();

  int readFromUri(const std::string& uri);
  int readFromFile(const std::string& file_name);
  bool preload(const std::string& file_name);
  void save(const std::string& file_name = "");

  bool isInit(bool print = true);
  void setLanguage(const std::string& language);
  std::string getLanguage();

  void setDisplay(bool display);

  ClassGraph class_graph_;
  ObjectPropertyGraph object_property_graph_;
  DataPropertyGraph data_property_graph_;
  IndividualGraph individual_graph_;
  AnonymousClassGraph anonymous_graph_;

private:
  OntologyLoader loader_;
  OntologyOwlWriter writer;

  bool is_preloaded_;
  bool is_init_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGY_H
