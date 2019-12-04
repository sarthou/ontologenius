#include "ontoloGenius/core/ontoGraphs/Ontology.h"

#include <iostream>

#include "ontoloGenius/core/ontoGraphs/Checkers/ClassChecker.h"
#include "ontoloGenius/core/ontoGraphs/Checkers/ObjectPropertyChecker.h"
#include "ontoloGenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"
#include "ontoloGenius/core/ontoGraphs/Checkers/IndividualChecker.h"

#include "ontoloGenius/graphical/Display.h"
#include "ontoloGenius/core/utility/error_code.h"

namespace ontologenius {

Ontology::Ontology(std::string language) : class_graph_(&individual_graph_, &object_property_graph_, &data_property_graph_),
                                           object_property_graph_(&class_graph_),
                                           data_property_graph_(&class_graph_),
                                           individual_graph_(&class_graph_, &object_property_graph_, &data_property_graph_),
                                           reader((Ontology&)*this),
                                           writer((Ontology&)*this)
{
  is_init_ = false;
  is_preloaded_ = false;
  class_graph_.setLanguage(language);
  object_property_graph_.setLanguage(language);
  data_property_graph_.setLanguage(language);
  individual_graph_.setLanguage(language);
  writer.setFileName("none");
}

Ontology::Ontology(const Ontology& other) : class_graph_(other.class_graph_, &individual_graph_, &object_property_graph_, &data_property_graph_),
                                            object_property_graph_(other.object_property_graph_, &class_graph_),
                                            data_property_graph_(other.data_property_graph_, &class_graph_),
                                            individual_graph_(other.individual_graph_, &class_graph_, &object_property_graph_, &data_property_graph_),
                                            reader((Ontology&)*this),
                                            writer((Ontology&)*this)
{
  class_graph_.deepCopy(other.class_graph_);
  object_property_graph_.deepCopy(other.object_property_graph_);
  data_property_graph_.deepCopy(other.data_property_graph_);
  individual_graph_.deepCopy(other.individual_graph_);

  is_init_ = true;
  is_preloaded_ = true;
  writer.setFileName("none");
}

Ontology::~Ontology()
{
  save();
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

  Display::info("\n***************SUMMARY****************");
  if(is_init_)
    Display::info("Ontology is closed :");
  else
    Display::warning("Ontology is not closed :");

  class_checker.printStatus();
  object_property_checker.printStatus();
  data_property_checker.printStatus();
  individual_checker.printStatus();
  Display::info("**************************************");

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

int Ontology::readFromFile(std::string file_name)
{
  files_.push_back(file_name);
  return reader.readFromFile(file_name);
}

bool Ontology::preload(std::string file_name)
{
  writer.setFileName(file_name);
  if(file_name != "none")
  {
    if(reader.readFromFile(file_name) == NO_ERROR)
      if(reader.readFromFile(file_name, true) == NO_ERROR)
        if(reader.empty() == false)
        {
          is_preloaded_ = true;
          Display::success("Ontology has been preloaded :");
          Display::success("ontoloGenius will NOT consider your default files");
          return true;
        }
  }

  Display::warning("Nothing to preload :");
  Display::warning("ontoloGenius will consider your default files");
  return false;
}

void Ontology::save(const std::string& file_name)
{
  if(!is_init_)
  {
    Display::error("Can not save unclosed ontology");
    return;
  }

  std::string tmp_name = writer.getFileName();

  if(file_name != "")
    writer.setFileName(file_name);

  writer.write();

  writer.setFileName(tmp_name);
}

bool Ontology::isInit(bool print)
{
  if(is_init_ == false)
    if(print == true)
      Display::error("Ontology is not closed");
  return is_init_;
}

void Ontology::setLanguage(std::string language)
{
  class_graph_.setLanguage(language);
  object_property_graph_.setLanguage(language);
  data_property_graph_.setLanguage(language);
  individual_graph_.setLanguage(language);
}

std::string Ontology::getLanguage()
{
  return class_graph_.getLanguage();
}

} // namespace ontologenius
