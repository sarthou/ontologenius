#include "ontologenius/core/ontologyIO/OntologyReader.h"

#include "ontologenius/core/ontoGraphs/Ontology.h"

namespace ontologenius {

OntologyReader::OntologyReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
  individual_graph_ = individual_graph;
  anonymous_graph_= anonymous_graph;
  nb_loaded_elem_ = 0;
  display_ = false;
}

OntologyReader::OntologyReader(Ontology& onto)
{
  class_graph_ = &onto.class_graph_;
  object_property_graph_ = &onto.object_property_graph_;
  individual_graph_ = &onto.individual_graph_;
  data_property_graph_ = &onto.data_property_graph_;
  anonymous_graph_= &onto.anonymous_graph_;
  nb_loaded_elem_ = 0;
  display_ = false;
}

} // namespace ontologenius