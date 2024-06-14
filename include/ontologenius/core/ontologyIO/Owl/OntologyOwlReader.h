#ifndef ONTOLOGENIUS_ONTOLOGYOWLREADER_H
#define ONTOLOGENIUS_ONTOLOGYOWLREADER_H

#include <iostream>
#include <map>
#include <string>
#include <tinyxml.h>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontologyIO/OntologyReader.h"

namespace ontologenius {

  class Ontology;

  class OntologyOwlReader : public OntologyReader
  {
  public:
    OntologyOwlReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph);
    explicit OntologyOwlReader(Ontology& onto);
    ~OntologyOwlReader() = default;

    int readFromUri(std::string content, const std::string& uri, bool individual = false);
    int readFromFile(const std::string& file_name, bool individual = false);

    std::vector<std::string> getImportsFromRaw(std::string content);
    std::vector<std::string> getImportsFromFile(const std::string& file_name);

    void setDisplay(bool display) { display_ = display; }
    void displayIndividualRules();
    bool empty() const { return (nb_loaded_elem_ == 0); }

  private:
    std::unordered_map<std::string, std::string> card_map_;
    int read(TiXmlElement* rdf, const std::string& name);
    int readIndividual(TiXmlElement* rdf, const std::string& name);

    void readClass(TiXmlElement* elem);
    void readEquivalentClass(AnonymousClassVectors_t& ano, TiXmlElement* elem, const std::string& class_name);

    ExpressionMember_t* readRestriction(TiXmlElement* elem);
    ExpressionMember_t* readClassExpression(TiXmlElement* elem);
    ExpressionMember_t* readDatatypeExpression(TiXmlElement* elem);
    ExpressionMember_t* readIntersection(TiXmlElement* elem);
    ExpressionMember_t* readUnion(TiXmlElement* elem);
    ExpressionMember_t* readOneOf(TiXmlElement* elem);
    ExpressionMember_t* readComplement(TiXmlElement* elem);
    ExpressionMember_t* readComplexDescription(TiXmlElement* elem);
    ExpressionMember_t* readResource(TiXmlElement* elem, const std::string& attribute_name = "rdf:resource");

    void addChildMember(ExpressionMember_t* parent, ExpressionMember_t* child, TiXmlElement* used_elem);

    bool readCardinalityRange(TiXmlElement* elem, ExpressionMember_t* exp);
    void readCardinalityValue(TiXmlElement* elem, ExpressionMember_t* exp);

    void readIndividual(TiXmlElement* elem);
    void readDescription(TiXmlElement* elem);
    void readIndividualDescription(TiXmlElement* elem);
    void readObjectProperty(TiXmlElement* elem);
    void readDataProperty(TiXmlElement* elem);
    void readAnnotationProperty(TiXmlElement* elem);
    void readCollection(std::vector<std::string>& vect, TiXmlElement* elem, const std::string& symbol, size_t level = 1);
    std::string readSomeValuesFrom(TiXmlElement* elem);

    inline void push(std::vector<std::string>& vect, TiXmlElement* sub_elem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    inline void push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole = "");
    inline void push(std::vector<SingleElement<std::string>>& vect, TiXmlElement* sub_elem, float probability, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    inline void push(std::vector<bool>& vect, bool elem, const std::string& symbole = "");
    void push(Properties_t& properties, TiXmlElement* sub_elem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    void pushLang(std::map<std::string, std::vector<std::string>>& dictionary, TiXmlElement* sub_elem);
    inline std::string getName(const std::string& uri);
    inline float getProbability(TiXmlElement* elem);
    inline std::string getAttribute(TiXmlElement* elem, const std::string& attribute);
    inline bool testAttribute(TiXmlElement* sub_elem, const std::string& attribute);
    inline int getNbChildren(TiXmlElement* elem);

    std::string toString(TiXmlElement* sub_elem, const std::string& attribute = "rdf:resource")
    {
      const char* sub_attr = sub_elem->Attribute(attribute.c_str());
      if(sub_attr != nullptr)
        return getName(std::string(sub_attr));
      return "";
    }

    void removeDocType(std::string& txt);
  };

  void OntologyOwlReader::push(std::vector<std::string>& vect, TiXmlElement* sub_elem, const std::string& symbole, const std::string& attribute)
  {
    std::string data = getAttribute(sub_elem, attribute);
    if(data.empty() == false)
    {
      vect.push_back(data);
      if(symbole.empty() == false && display_)
        std::cout << "│   │   ├── " << symbole << " " << data << std::endl;
    }
  }

  void OntologyOwlReader::push(std::vector<SingleElement<std::string>>& vect, TiXmlElement* sub_elem, float probability, const std::string& symbole, const std::string& attribute)
  {
    std::string data = getAttribute(sub_elem, attribute);
    if(data.empty() == false)
    {
      vect.emplace_back(data, probability);
      if(symbole.empty() == false && display_)
        std::cout << "│   │   ├── " << symbole << " " << data << std::endl;
    }
  }

  void OntologyOwlReader::push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole)
  {
    const std::string& data = elem;
    if(data.empty() == false)
    {
      vect.push_back(data);
      if((symbole.empty() == false) && display_)
        std::cout << "│   │   ├── " << symbole << " " << data << std::endl;
    }
  }
  std::string OntologyOwlReader::getName(const std::string& uri)
  {
    size_t pos = uri.find('#');
    std::string result = uri.substr(pos + 1);
    return result;
  }

  float OntologyOwlReader::getProbability(TiXmlElement* elem)
  {
    float proba = 1.0;

    const char* sub_attr = elem->Attribute("onto:probability");
    if(sub_attr != nullptr)
      proba = std::stof(std::string(sub_attr));

    return proba;
  }

  inline std::string OntologyOwlReader::getAttribute(TiXmlElement* elem, const std::string& attribute)
  {
    const char* sub_attr = elem->Attribute(attribute.c_str());
    if(sub_attr != nullptr)
      return getName(std::string(sub_attr));
    else
      return "";
  }

  bool OntologyOwlReader::testAttribute(TiXmlElement* sub_elem, const std::string& attribute)
  {
    const char* sub_attr = sub_elem->Attribute(attribute.c_str());
    if(sub_attr != nullptr)
      return true;
    else
      return false;
  }

  int OntologyOwlReader::getNbChildren(TiXmlElement* elem)
  {
    int cpt = 0;
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      cpt++;
    return cpt;
  }
} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYOWLREADER_H
