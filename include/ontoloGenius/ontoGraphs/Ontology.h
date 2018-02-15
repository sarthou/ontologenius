#ifndef ONTOLOGY_H
#define ONTOLOGY_H

#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/PropertyGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/IndividualGraph.h"

#include "ontoloGenius/ontoGraphs/OntologyReader.h"

class Ontology
{
public:
  Ontology() : properties_(&classes_), individuals_(&classes_, &properties_), reader((Ontology&)*this) {}
  ~Ontology() {}

  int close();

  int readFromUri(std::string uri);
  int readFromFile(std::string fileName);

  ClassGraph classes_;
  PropertyGraph properties_;
  IndividualGraph individuals_;

private:
  OntologyReader reader;
  std::vector<std::string> files_;
  std::vector<std::string> uri_;
};


#endif
