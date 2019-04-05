#ifndef ONTOLOGENIUS_ONTOLOGYDRAWER_H
#define ONTOLOGENIUS_ONTOLOGYDRAWER_H

#include <string>

#include "ontoloGenius/core/ontoGraphs/Ontology.h"

#include "ontoloGenius/graphical/Drawers/ClassDrawer.h"
#include "ontoloGenius/graphical/Drawers/ObjectPropertyDrawer.h"
#include "ontoloGenius/graphical/Drawers/DataPropertyDrawer.h"

namespace ontologenius {

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

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYDRAWER_H
