#ifndef ONTOLOGY_H
#define ONTOLOGY_H

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include "ontoloGenius/core/ontologyIO/OntologyReader.h"
#include "ontoloGenius/core/ontologyIO/OntologyWriter.h"
#include <iostream>

namespace ontologenius {

class Ontology
{
public:
  Ontology(std::string language = "en");
  ~Ontology();

  int close();

  int readFromUri(std::string uri);
  int readFromFile(std::string fileName);
  bool preload(std::string fileName);

  bool isInit(bool print = true);
  void setLanguage(std::string language);
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

#endif
