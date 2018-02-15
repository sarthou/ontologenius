#include "ontoloGenius/ontoGraphs/Ontology.h"

#include "ontoloGenius/ontoGraphs/Checkers/ClassChecker.h"
#include "ontoloGenius/ontoGraphs/Checkers/PropertyChecker.h"

int Ontology::close()
{
  classes_.close();
  properties_.close();
  individuals_.close();

  ClassChecker classChecker(&classes_);
  PropertyChecker propertyChecker(&properties_);

  size_t err = classChecker.check();
  err += propertyChecker.check();

  if(err)
    return -1;
  else
  {
    reader.displayIndividualRules();
    
    for(size_t i = 0; i < uri_.size(); i++)
      reader.readFromUri(uri_[i], true);
    uri_.clear();

    for(size_t i = 0; i < files_.size(); i++)
      reader.readFromFile(files_[i], true);
    files_.clear();

    return 0;
  }
}

int Ontology::readFromUri(std::string uri)
{
  uri_.push_back(uri);
  reader.readFromUri(uri);
}

int Ontology::readFromFile(std::string fileName)
{
  files_.push_back(fileName);
  reader.readFromFile(fileName);
}
