#ifndef ONTOLOGENIUS_ONTOLOGYDRAWER_H
#define ONTOLOGENIUS_ONTOLOGYDRAWER_H

#include <string>

#include "ontologenius/core/ontoGraphs/Ontology.h"

#include "ontologenius/graphical/Drawers/ClassDrawer.h"
#include "ontologenius/graphical/Drawers/ObjectPropertyDrawer.h"
#include "ontologenius/graphical/Drawers/DataPropertyDrawer.h"

namespace ontologenius {

class OntologyDrawer
{
public:
  OntologyDrawer(Ontology* ontology = nullptr);
  ~OntologyDrawer() {}

  void setOntology(Ontology* ontology);
  void draw(const std::string& name);

private:
  ClassDrawer class_drawer_;
  ObjectPropertyDrawer object_property_drawer_;
  DataPropertyDrawer data_property_drawer_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYDRAWER_H
