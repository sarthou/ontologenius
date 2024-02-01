#include "ontologenius/core/ontologyIO/OntologyLoader.h"

#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"
#include "ontologenius/utils/String.h"
#include "ontologenius/utils/Commands.h"

#include <curl/curl.h>

namespace ontologenius {

OntologyLoader::OntologyLoader(ClassGraph* class_graph,
                               ObjectPropertyGraph* object_property_graph,
                               DataPropertyGraph* data_property_graph,
                               IndividualGraph* individual_graph,
                               AnonymousClassGraph* anonymous_graph) : owl_reader_(class_graph, object_property_graph, data_property_graph, individual_graph, anonymous_graph),
                                                                    ttl_reader_(class_graph, object_property_graph, data_property_graph, individual_graph, anonymous_graph)
{}

OntologyLoader::OntologyLoader(Ontology& onto) : owl_reader_(onto), ttl_reader_(onto)
{}

int OntologyLoader::loadFile(std::string file)
{
  fixPath(file);
  if(std::find(loading_files_.begin(), loading_files_.end(), file) == loading_files_.end())
  {
    loading_files_.push_back(file);
    if(file.find(".ttl") == std::string::npos)
    {
      auto imports = owl_reader_.getImportsFromFile(file);
      loadImports(imports);

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
  if(std::find(loading_uri_.begin(), loading_uri_.end(), uri) == loading_uri_.end())
  {
    loading_uri_.push_back(uri);

    int download_err = downloadFile(uri);
    if(download_err == NO_ERROR)
    {
      if(uri.find(".ttl") == std::string::npos)
      {
        auto imports = owl_reader_.getImportsFromRaw(uri_to_file_[uri]);
        loadImports(imports);

        int err = owl_reader_.readFromUri(uri_to_file_[uri], uri);
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
      return download_err;
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
      err += owl_reader_.readFromUri(uri_to_file_[uri], uri, true);
    else
      err += ttl_reader_.readFromUri(uri_to_file_[uri], uri);
  }
  uri_.clear();
  uri_to_file_.clear();

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

bool OntologyLoader::isProtected(const std::string& page_content)
{
  if(page_content.find("type=\"password\"") != std::string::npos)
    return true;
  else
    return false;
}

int OntologyLoader::downloadFile(const std::string& uri)
{
  std::string response = downlaodFileCurl(uri);

  if(isProtected(response))
  {
    Display::warning("The requested file may be protected: " + uri);
    return REQUEST_ERROR;
  }
  else
  {
    uri_to_file_.emplace(uri, response);
    return NO_ERROR;
  }
}

std::string OntologyLoader::downlaodFileCurl(const std::string& uri)
{
  std::string res;
  res.reserve(1024*1024);
  CURL* curl_handle;
  curl_global_init(CURL_GLOBAL_ALL);
  curl_handle = curl_easy_init();
  curl_easy_setopt(curl_handle, CURLOPT_URL, uri.c_str());
  curl_easy_setopt(curl_handle, CURLOPT_VERBOSE, 1L);
  curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, 
    +[](void *ptr, size_t size, size_t nmemb, void *stream) -> size_t // the + forces the cast into void*
    {
      if(stream != nullptr) {
        ((std::string*)stream)->append((const char*)ptr, size*nmemb);
        return size*nmemb;
      }
      else
        return 0;
    });
  curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &res);
  curl_easy_perform(curl_handle);
  curl_easy_cleanup(curl_handle);
  curl_global_cleanup();
  return res;
}

void OntologyLoader::loadImports(const std::vector<std::string>& imports)
{
  for(auto& import : imports)
  {
    bool loaded = true;
    auto with_package = resolvePath(import);
    if(with_package.first != "")
    {
      std::string path = findPackage(with_package.first);
      path += "/" + with_package.second;

      fixPath(path);
      if(std::find(loading_files_.begin(), loading_files_.end(), path) == loading_files_.end())
      {
        Display::info("[OntologyLoader] Package " + with_package.first + " has been detected");
        Display::info("[OntologyLoader] Load imported file: " + path);
        if(loadFile(path) != NO_ERROR)
        {
          Display::warning("[OntologyLoader] Automatic package deduction failed.");
          loaded = false;
        }
      }
    }
    else
      loaded = false;
    
    if(!loaded && import.find("http") == 0)
    {
      std::string path = import;
      fixUrl(path);
      if(std::find(loading_uri_.begin(), loading_uri_.end(), path) == loading_uri_.end())
      {
        Display::info("[OntologyLoader] Load imported uri: " + path);
        loadUri(path);
      }
    }
    else if(!loaded)
    {
      std::string path = import;
      fixPath(path);
      if(std::find(loading_files_.begin(), loading_files_.end(), path) == loading_files_.end())
      {
        Display::info("[OntologyLoader] Load imported file: " + path);
        loadFile(path);
      }
    }
  }
}

std::pair<std::string, std::string> OntologyLoader::resolvePath(const std::string& raw_path)
{
  std::vector<std::string> packages = listPackages();

  auto parts = split(raw_path, "/");
  for(auto part_it = parts.begin(); part_it != parts.end(); ++part_it)
  {
    auto package_it = std::find(packages.begin(), packages.end(), *part_it);
    if(package_it != packages.end())
    {
      size_t package_pose = raw_path.find(*part_it);
      std::string rest = raw_path.substr(package_pose + (*part_it).size() + 1);

      // The following is used to remove extras added by github and gitlab

      size_t useless_pose = rest.find("blob/");
      if(useless_pose != std::string::npos)
      {
        useless_pose = rest.find("/", useless_pose + std::string("blob/").size() + 1);
        rest = rest.substr(useless_pose + 1);
      }

      useless_pose = rest.find("raw/");
      if(useless_pose != std::string::npos)
      {
        useless_pose = rest.find("/", useless_pose + std::string("raw/").size() + 1);
        rest = rest.substr(useless_pose + 1);
      }

      useless_pose = rest.find("raw.githubusercontent.com/");
      if(useless_pose != std::string::npos)
      {
        useless_pose = rest.find("/");
        rest = rest.substr(useless_pose + 1);
      }

      return {*part_it, rest};
    }
  }

  return {"", raw_path};
}

} // namespace eontologenius