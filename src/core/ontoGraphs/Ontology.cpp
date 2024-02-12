#include "ontologenius/core/ontoGraphs/Ontology.h"

#include <iostream>

#include "ontologenius/core/ontoGraphs/Checkers/ClassChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/IndividualChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/ObjectPropertyChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/AnonymousClassChecker.h"

#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

Ontology::Ontology(const std::string& language) : class_graph_(&individual_graph_, &object_property_graph_, &data_property_graph_),
                                                  object_property_graph_(&individual_graph_, &class_graph_),
                                                  data_property_graph_(&individual_graph_, &class_graph_),
                                                  individual_graph_(&class_graph_, &object_property_graph_, &data_property_graph_),
                                                  anonymous_graph_(&class_graph_, &object_property_graph_, &data_property_graph_, &individual_graph_),
                                                  loader_((Ontology&)*this),
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
                                            object_property_graph_(other.object_property_graph_, &individual_graph_, &class_graph_),
                                            data_property_graph_(other.data_property_graph_, &individual_graph_, &class_graph_),
                                            individual_graph_(other.individual_graph_, &class_graph_, &object_property_graph_, &data_property_graph_),
                                            anonymous_graph_(other.anonymous_graph_, &class_graph_, &object_property_graph_, &data_property_graph_, &individual_graph_),
                                            loader_((Ontology&)*this),
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

bool Ontology::close()
{
  if(is_init_ == true)
    return true;

  ClassChecker class_checker(&class_graph_);
  ObjectPropertyChecker object_property_checker(&object_property_graph_);
  DataPropertyChecker data_property_checker(&data_property_graph_);
  IndividualChecker individual_checker(&individual_graph_);
  AnonymousClassChecker ano_class_checker(&anonymous_graph_);

  size_t err = class_checker.check();
  err += object_property_checker.check();
  err += data_property_checker.check();

  if(err == 0)
  {
    loader_.loadIndividuals();

    individual_checker = IndividualChecker(&individual_graph_);
    err += individual_checker.check();

    is_init_ = true;
  }
  err += ano_class_checker.check();

  Display::info("\n***************SUMMARY****************");
  if(is_init_)
    Display::info("Ontology is closed :");
  else
    Display::warning("Ontology is not closed :");

  class_checker.printStatus();
  object_property_checker.printStatus();
  data_property_checker.printStatus();
  individual_checker.printStatus();
  ano_class_checker.printStatus();

  Display::info("**************************************");

  if(err)
    return false;
  else
    return true;
}

int Ontology::readFromUri(const std::string& uri)
{
  return loader_.loadUri(uri);
}

int Ontology::readFromFile(const std::string& file_name)
{
  return loader_.loadFile(file_name);
}

bool Ontology::preload(const std::string& file_name)
{
  writer.setFileName(file_name);
  if(file_name != "none")
  {
    if(loader_.loadFile(file_name) == NO_ERROR)
      if(loader_.loadIndividuals() == NO_ERROR)
        if(loader_.isEmpty() == false)
        {
          is_preloaded_ = true;
          Display::success("Ontology has been preloaded");
          Display::success("Ontologenius will NOT consider your default files");
          return true;
        }
  }

  Display::warning("Nothing to preload :");
  Display::warning("ontologenius will consider your default files");
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

void Ontology::setLanguage(const std::string& language)
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

void Ontology::setDisplay(bool display)
{
  loader_.setDisplay(display);
}

} // namespace ontologenius
