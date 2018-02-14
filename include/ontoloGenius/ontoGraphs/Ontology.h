#ifndef ONTOLOGY_H
#define ONTOLOGY_H

#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/PropertyGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/IndividualGraph.h"

class Ontology
{
public:
  Ontology() : properties_(&classes_), individuals_(&classes_, &properties_) {}
  ~Ontology() {}

  int close();

  ClassGraph classes_;
  PropertyGraph properties_;
  IndividualGraph individuals_;
};


#endif
