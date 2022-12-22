#include "ontologenius/core/ontologyIO/OntologyLoader.h"
#include "ontologenius/core/utility/error_code.h"

namespace ontologenius {

OntologyLoader::OntologyLoader(ClassGraph* class_graph,
                               ObjectPropertyGraph* object_property_graph,
                               DataPropertyGraph* data_property_graph,
                               IndividualGraph* individual_graph) : owl_reader_(class_graph, object_property_graph, data_property_graph, individual_graph),
                                                                    ttl_reader_(class_graph, object_property_graph, data_property_graph, individual_graph)
{}

OntologyLoader::OntologyLoader(Ontology& onto) : owl_reader_(onto), ttl_reader_(onto)
{}

int OntologyLoader::loadFile(const std::string& file)
{
  if(file.find(".ttl") == std::string::npos)
  {
    int err = owl_reader_.readFromFile(file);
    if(err == NO_ERROR)
      files_.push_back(file);
    return err;
  }
  else
  {
    files_.push_back(file);
    return NO_ERROR; // ttl files only describe individuals
  }
}

int OntologyLoader::loadUri(const std::string& uri)
{
  if(uri.find(".ttl") == std::string::npos)
  {
    int err = owl_reader_.readFromUri(uri);
    if(err == NO_ERROR)
      uri_.push_back(uri);
    return err;
  }
  else
  {
    uri_.push_back(uri);
    return NO_ERROR; // ttl files only describe individuals
  }
}

int OntologyLoader::loadIndividuals()
{
  int err = NO_ERROR;
  owl_reader_.displayIndividualRules();

  for(auto& uri : uri_)
  {
    if(uri.find(".ttl") == std::string::npos)
      err += owl_reader_.readFromUri(uri, true);
    else
      err += ttl_reader_.readFromUri(uri);
  }
  uri_.clear();

  for(auto& file : files_)
  {
    if(file.find(".ttl") == std::string::npos)
      err += owl_reader_.readFromFile(file, true);
    else
      err += ttl_reader_.readFromFile(file);
  }
  files_.clear();

  if(err != NO_ERROR)
    err = OTHER;
  return err;
}

int OntologyLoader::getNbLoadedElements()
{
  return owl_reader_.getNbLoadedElements() + ttl_reader_.getNbLoadedElements();
}

void OntologyLoader::setDisplay(bool display)
{
  owl_reader_.setDisplay(display);
  ttl_reader_.setDisplay(display);
}

} // namespace eontologenius