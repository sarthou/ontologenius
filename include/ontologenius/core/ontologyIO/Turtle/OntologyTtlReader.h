#ifndef ONTOLOGENIUS_ONTOLOGYTTLREADER_H
#define ONTOLOGENIUS_ONTOLOGYTTLREADER_H

#include "ontologenius/core/ontologyIO/OntologyReader.h"

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include <regex>

namespace ontologenius {

class OntologyTtlReader : public OntologyReader
{
public:
  OntologyTtlReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph);
  explicit OntologyTtlReader(Ontology& onto);
  ~OntologyTtlReader() {}

  int readFromUri(std::string& content, const std::string& uri);
  int readFromFile(const std::string& fileName);

private:
  std::string previous_subject_;
  IndividualVectors_t individual_vector_;

  std::regex double_reg_;
  std::regex decimal_reg_;
  std::regex integer_reg_;

  int read(std::string& raw_turtle, const std::string& file_name);

  void removeComments(std::string& raw_turtle);
  void readTriplets(const std::string& raw_turtle);
  void sendToOntology(std::string& subject, const std::vector<std::array<std::string,3>>& triplets);
  bool isMultiLineDelimiter(const std::string& raw_turtle, size_t& pose, char delim);
  bool isDelimiterEscaped(const std::string& raw_turtle, size_t& pose);
  inline size_t nextNonBlanckCharacter(const std::string& text, size_t pose);
  inline size_t nextBlanckCharacter(const std::string& text, size_t pose);
  size_t endOfBlock(const std::string& text, size_t pose);
  std::string getElement(const std::string& text, size_t pose);

  std::string getSubject(const std::string& element);
  std::string getProperty(const std::string& element);
  std::pair<std::string, std::string> getObject(const std::string& element);

  inline void push(std::vector<Single_t<std::string>>& vect, const std::string& element, float probability, const std::string& symbole);
  inline void pushLang(std::map<std::string, std::vector<std::string>>& dictionary, const std::pair<std::string, std::string>& label);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYTTLREADER_H