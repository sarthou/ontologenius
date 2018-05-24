#ifndef ONTOLOGYDRAWER_H
#define ONTOLOGYDRAWER_H

#include "ontoloGenius/core/ontoGraphs/Ontology.h"

#include "ontoloGenius/graphical/Drawers/ClassDrawer.h"
#include "ontoloGenius/graphical/Drawers/ObjectPropertyDrawer.h"
#include "ontoloGenius/graphical/Drawers/DataPropertyDrawer.h"

#include <string>

class OntologyDrawer
{
public:
  OntologyDrawer(Ontology* ontology = nullptr);
  ~OntologyDrawer() {}

  void setOntology(Ontology* ontology);
  void draw(std::string name);

private:
  ClassDrawer class_drawer_;
  ObjectPropertyDrawer object_property_drawer_;
  DataPropertyDrawer data_property_drawer_;
};

#endif
