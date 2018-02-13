#ifndef ONTOLOGY_H
#define ONTOLOGY_H

#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/PropertyGraph.h"

class Ontology
{
public:
  Ontology() : properties_(&classes_) {}
  ~Ontology() {}

  int close();

  ClassGraph classes_;
  PropertyGraph properties_;
};


#endif
