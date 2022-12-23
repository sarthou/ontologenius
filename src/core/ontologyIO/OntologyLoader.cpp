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

int OntologyLoader::loadFile(std::string file)
{
  fixPath(file);
  if(std::find(files_.begin(), files_.end(), file) == files_.end())
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
  else
    return NO_ERROR;
}

int OntologyLoader::loadUri(std::string uri)
{
  fixUrl(uri);
  if(std::find(uri_.begin(), uri_.end(), uri) == uri_.end())
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
  else
    return NO_ERROR;
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

void OntologyLoader::fixUrl(std::string& url)
{
  size_t dot_pose = url.find_last_of(".");
  size_t pose = url.find_last_of("/");
  if(dot_pose < pose)
    url += ".owl";

  pose = url.find("github.");
  if(pose != std::string::npos)
  {
    url.replace(pose, std::string("github.").size(), "raw.githubusercontent.");
    size_t blob_pose = url.find("blob/");
    url.erase(blob_pose, std::string("blob/").size());
  }

  pose = url.find("gitlab.");
  if(pose != std::string::npos)
  {
    pose = url.find("/blob/");
    url.replace(pose, std::string("/blob/").size(), "/raw/");
  }
}

void OntologyLoader::fixPath(std::string& path)
{
  size_t dot_pose = path.find_last_of(".");
  if(dot_pose == std::string::npos)
    path += ".owl";
}

} // namespace eontologenius