#ifndef ONTOLOGENIUS_ONTOLOGY_H
#define ONTOLOGENIUS_ONTOLOGY_H

#include <string>

#include "ontologenius/core/ontoGraphs/Graphs/OntologyGraphs.h"
#include "ontologenius/core/ontologyIO/OntologyLoader.h"
#include "ontologenius/core/ontologyIO/Owl/OntologyOwlWriter.h"

namespace ontologenius {

  class Ontology : public OntologyGraphs
  {
  public:
    Ontology(const std::string& language = "en");
    Ontology(const Ontology& other);
    ~Ontology();

    bool close();

    int readFromUri(const std::string& uri);
    int readFromFile(const std::string& file_name);
    bool preload(const std::string& file_name);
    void save(const std::string& file_name = "");

    bool isInit(bool print = true) const;
    void setLanguage(const std::string& language);
    std::string getLanguage() const;

    void setDisplay(bool display);

  private:
    OntologyLoader loader_;
    OntologyOwlWriter writer_;

    bool is_init_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGY_H
