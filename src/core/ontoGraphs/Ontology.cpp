#include "ontologenius/core/ontoGraphs/Ontology.h"

#include <cstddef>
#include <string>

#include "ontologenius/core/ontoGraphs/Checkers/AnonymousClassChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/ClassChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/DataPropertyChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/IndividualChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/ObjectPropertyChecker.h"
#include "ontologenius/core/ontoGraphs/Checkers/RuleChecker.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntologyGraphs.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

  Ontology::Ontology(const std::string& language) : loader_(this),
                                                    writer_(this),
                                                    is_preloaded_(false),
                                                    is_init_(false)
  {
    classes_.setLanguage(language);
    object_properties_.setLanguage(language);
    data_properties_.setLanguage(language);
    individuals_.setLanguage(language);
    writer_.setFileName("none");
  }

  Ontology::Ontology(const Ontology& other) : OntologyGraphs(other),
                                              loader_(this),
                                              writer_(this),
                                              is_preloaded_(true),
                                              is_init_(true)
  {
    writer_.setFileName("none");
  }

  Ontology::~Ontology()
  {
    save();
  }

  bool Ontology::close()
  {
    if(is_init_ == true)
      return true;

    ClassChecker class_checker(&classes_, this);
    ObjectPropertyChecker object_property_checker(&object_properties_, this);
    DataPropertyChecker data_property_checker(&data_properties_, this);
    IndividualChecker individual_checker(&individuals_, this);
    AnonymousClassChecker ano_class_checker(&anonymous_classes_, this);
    RuleChecker rule_checker(&rules_, this);

    size_t err = class_checker.check();
    err += object_property_checker.check();
    err += data_property_checker.check();

    if(err == 0)
    {
      anonymous_classes_.analyseApplicabiltiy();

      loader_.loadIndividuals();

      individual_checker = IndividualChecker(&individuals_, this); // We need to re-init the checker with the read graph

      err += individual_checker.check();

      is_init_ = true;
    }
    err += ano_class_checker.check();
    err += rule_checker.check();

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
    rule_checker.printStatus();

    Display::info("**************************************");

    if(err != 0)
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
    writer_.setFileName(file_name);
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

    const std::string tmp_name = writer_.getFileName();

    if(file_name.empty() == false)
      writer_.setFileName(file_name);

    writer_.write();

    writer_.setFileName(tmp_name);
  }

  bool Ontology::isInit(bool print) const
  {
    if(is_init_ == false)
      if(print == true)
        Display::error("Ontology is not closed");
    return is_init_;
  }

  void Ontology::setLanguage(const std::string& language)
  {
    classes_.setLanguage(language);
    object_properties_.setLanguage(language);
    data_properties_.setLanguage(language);
    individuals_.setLanguage(language);
  }

  std::string Ontology::getLanguage() const
  {
    return classes_.getLanguage();
  }

  void Ontology::setDisplay(bool display)
  {
    loader_.setDisplay(display);
  }

} // namespace ontologenius
