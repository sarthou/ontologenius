#include "ontoloGenius/graphical/Drawers/OntologyDrawer.h"

namespace ontologenius {

OntologyDrawer::OntologyDrawer(Ontology* ontology)
{
  if(ontology != nullptr)
  {
    class_drawer_.setGraph(&ontology->class_graph_);
    object_property_drawer_.setGraph(&ontology->object_property_graph_);
    data_property_drawer_.setGraph(&ontology->data_property_graph_);
  }
}

void OntologyDrawer::setOntology(Ontology* ontology)
{
  if(ontology != nullptr)
  {
    class_drawer_.setGraph(&ontology->class_graph_);
    object_property_drawer_.setGraph(&ontology->object_property_graph_);
    data_property_drawer_.setGraph(&ontology->data_property_graph_);
  }
}

void OntologyDrawer::draw(std::string name)
{
  class_drawer_.putInLayers();
  class_drawer_.draw(name + "_class.png");

  object_property_drawer_.putInLayers();
  object_property_drawer_.draw(name + "_objectProperty.png");

  data_property_drawer_.putInLayers();
  data_property_drawer_.draw(name + "_dataProperty.png");
}

} // namespace ontologenius
