#include "ontoloGenius/ontoGraphs/Ontology.h"

#include "ontoloGenius/ontoGraphs/Checkers/ClassChecker.h"
#include "ontoloGenius/ontoGraphs/Checkers/PropertyChecker.h"

#include <iostream>

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif

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

    is_init_ = true;
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

bool Ontology::isInit()
{
  if(is_init_ == false)
    std::cout << COLOR_RED << "Ontology is not closed" << COLOR_OFF << std::endl;
  return is_init_;
}
