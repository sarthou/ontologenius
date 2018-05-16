#include "ontoloGenius/ontoGraphs/Ontology.h"

#include "ontoloGenius/ontoGraphs/Checkers/ClassChecker.h"
#include "ontoloGenius/ontoGraphs/Checkers/ObjectPropertyChecker.h"
#include "ontoloGenius/ontoGraphs/Checkers/DataPropertyChecker.h"
#include "ontoloGenius/ontoGraphs/Checkers/IndividualChecker.h"

#include <iostream>

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif

Ontology::Ontology(std::string language) : object_property_graph_(&class_graph_),
                                           data_property_graph_(&class_graph_),
                                           individual_graph_(&class_graph_, &object_property_graph_, &data_property_graph_),
                                           reader((Ontology&)*this)
{
  is_init_ = false;
  class_graph_.setLanguage(language);
  object_property_graph_.setLanguage(language);
  data_property_graph_.setLanguage(language);
  individual_graph_.setLanguage(language);
}

int Ontology::close()
{
  if(is_init_ == true)
    return 0;

  class_graph_.close();
  object_property_graph_.close();
  data_property_graph_.close();

  ClassChecker class_checker(&class_graph_);
  ObjectPropertyChecker object_property_checker(&object_property_graph_);
  DataPropertyChecker data_property_checker(&data_property_graph_);
  IndividualChecker individual_checker(&individual_graph_);

  size_t err = class_checker.check();
  err += object_property_checker.check();
  err += data_property_checker.check();

  if(err == 0)
  {
    reader.displayIndividualRules();

    for(size_t i = 0; i < uri_.size(); i++)
      reader.readFromUri(uri_[i], true);
    uri_.clear();

    for(size_t i = 0; i < files_.size(); i++)
      reader.readFromFile(files_[i], true);
    files_.clear();

    individual_graph_.close();

    individual_checker = IndividualChecker(&individual_graph_);
    err += individual_checker.check();

    is_init_ = true;
  }

  std::cout << std::endl << std::endl << "***************SUMMARY****************" << std::endl;
  if(is_init_)
    std::cout << "Ontology is closed :" << std::endl;
  else
    std::cout << "Ontology is not closed :" << std::endl;

  class_checker.printStatus();
  object_property_checker.printStatus();
  data_property_checker.printStatus();
  individual_checker.printStatus();
  std::cout << "**************************************" << std::endl;

  if(err)
    return -1;
  else
    return 0;
}

int Ontology::readFromUri(std::string uri)
{
  uri_.push_back(uri);
  return reader.readFromUri(uri);
}

int Ontology::readFromFile(std::string fileName)
{
  files_.push_back(fileName);
  return reader.readFromFile(fileName);
}

bool Ontology::isInit()
{
  if(is_init_ == false)
    std::cout << COLOR_RED << "Ontology is not closed" << COLOR_OFF << std::endl;
  return is_init_;
}
