#include "ontoloGenius/ontoGraphs/Drawers/OntologyDrawer.h"

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
  class_drawer_.put_in_layers();
  class_drawer_.draw(name + "_class.png");

  object_property_drawer_.put_in_layers();
  object_property_drawer_.draw(name + "_objectProperty.png");

  data_property_drawer_.put_in_layers();
  data_property_drawer_.draw(name + "_dataProperty.png");
}
