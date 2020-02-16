#ifndef ONTOLOGENIUS_ONTOLOGY_H
#define ONTOLOGENIUS_ONTOLOGY_H

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include "ontologenius/core/ontologyIO/OntologyReader.h"
#include "ontologenius/core/ontologyIO/OntologyWriter.h"

namespace ontologenius {

class Ontology
{
public:
  Ontology(std::string language = "en");
  Ontology(const Ontology& other);
  ~Ontology();

  int close();

  int readFromUri(const std::string& uri);
  int readFromFile(const std::string& file_name);
  bool preload(const std::string& file_name);
  void save(const std::string& file_name = "");

  bool isInit(bool print = true);
  void setLanguage(const std::string& language);
  std::string getLanguage();

  ClassGraph class_graph_;
  ObjectPropertyGraph object_property_graph_;
  DataPropertyGraph data_property_graph_;
  IndividualGraph individual_graph_;

private:
  OntologyReader reader;
  OntologyWriter writer;

  std::vector<std::string> files_;
  std::vector<std::string> uri_;

  bool is_preloaded_;
  bool is_init_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGY_H
