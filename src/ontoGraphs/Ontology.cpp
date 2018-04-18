#include "ontoloGenius/ontoGraphs/Ontology.h"

#include "ontoloGenius/ontoGraphs/Checkers/ClassChecker.h"
#include "ontoloGenius/ontoGraphs/Checkers/PropertyChecker.h"
#include "ontoloGenius/ontoGraphs/Checkers/IndividualChecker.h"

#include <iostream>

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif

Ontology::Ontology(std::string language) : properties_(&classes_), individuals_(&classes_, &properties_), reader((Ontology&)*this)
{
  is_init_ = false;
  classes_.setLanguage(language);
  properties_.setLanguage(language);
  individuals_.setLanguage(language);
}

int Ontology::close()
{
  if(is_init_ == true)
    return 0;

  classes_.close();
  properties_.close();

  ClassChecker classChecker(&classes_);
  PropertyChecker propertyChecker(&properties_);
  IndividualChecker individualChecker(&individuals_);

  size_t err = classChecker.check();
  err += propertyChecker.check();

  if(err == 0)
  {
    reader.displayIndividualRules();

    for(size_t i = 0; i < uri_.size(); i++)
      reader.readFromUri(uri_[i], true);
    uri_.clear();

    for(size_t i = 0; i < files_.size(); i++)
      reader.readFromFile(files_[i], true);
    files_.clear();

    individuals_.close();

    individualChecker = IndividualChecker(&individuals_);
    err += individualChecker.check();

    is_init_ = true;
  }

  std::cout << std::endl << std::endl << "***************SUMMARY****************" << std::endl;
  if(is_init_)
    std::cout << "Ontology is closed :" << std::endl;
  else
    std::cout << "Ontology is not closed :" << std::endl;

  classChecker.printStatus();
  propertyChecker.printStatus();
  individualChecker.printStatus();
  std::cout << "**************************************" << std::endl;

  if(err)
    return -1;
  else
    return 0;
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
