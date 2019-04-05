#ifndef ONTOLOGENIUS_ONTOLOGYREADER_H
#define ONTOLOGENIUS_ONTOLOGYREADER_H

#include <vector>
#include <string>
#include <map>
#include <iostream>

#include <tinyxml.h>

#include <ros/ros.h>

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

class Ontology;

class OntologyReader
{
public:
  OntologyReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
  OntologyReader(Ontology& onto);
  ~OntologyReader() {}

  int readFromUri(std::string& uri, bool individual = false);
  int readFromFile(std::string& fileName, bool individual = false);

  void displayIndividualRules();
  bool empty() {return (elemLoaded == 0); }

private:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;
  IndividualGraph* individual_graph_;

  int elemLoaded;

  int read(TiXmlElement* rdf, std::string& name);
  int readIndividual(TiXmlElement* rdf, std::string& name);

  void readClass(TiXmlElement* elem);
  void readIndividual(TiXmlElement* elem);
  void readDescription(TiXmlElement* elem);
  void readIndividualDescription(TiXmlElement* elem);
  void readObjectProperty(TiXmlElement* elem);
  void readDataProperty(TiXmlElement* elem);
  void readCollection(std::vector<std::string>& vect, TiXmlElement* elem, std::string symbol, size_t level = 1);
  void readRestriction(TiXmlElement* elem);
  std::string readSomeValuesFrom(TiXmlElement* elem);

  inline void push(std::vector<std::string>& vect, TiXmlElement* subElem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
  inline void push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole = "");
  inline void push(std::vector<bool>& vect, bool elem, const std::string& symbole = "");
  void push(Properties_t& properties, TiXmlElement* subElem, std::string symbole = "", std::string attribute = "rdf:resource");
  void pushLang(std::map<std::string, std::vector<std::string>>& dictionary, TiXmlElement* subElem);
  inline std::string getName(const std::string& uri);
  inline std::string getAttribute(TiXmlElement* elem, const std::string& attribute);
  inline bool testAttribute(TiXmlElement* subElem, const std::string& attribute);
};

void OntologyReader::push(std::vector<std::string>& vect, TiXmlElement* subElem, const std::string& symbole, const std::string& attribute)
{
  const char* subAttr;
  subAttr = subElem->Attribute(attribute.c_str());
  if(subAttr != NULL)
  {
    vect.push_back(getName(std::string(subAttr)));
    std::cout << "│   │   ├── " << symbole << getName(std::string(subAttr)) << std::endl;
  }
  else
  {
    for(TiXmlElement* subsubElem = subElem->FirstChildElement(); subsubElem != NULL; subsubElem = subsubElem->NextSiblingElement())
    {
      std::string name = subsubElem->Value();
      if(name == "owl:Restriction")
        readRestriction(subsubElem);
    }

  }
}

void OntologyReader::push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole)
{
  vect.push_back(elem);
  if(symbole != "")
    std::cout << "│   │   ├── " << symbole << elem << std::endl;
}

void OntologyReader::push(std::vector<bool>& vect, bool elem, const std::string& symbole)
{
  vect.push_back(elem);
  if(symbole != "")
  {
    if(elem == true)
      std::cout << "│   │   ├── " << symbole << " true" << std::endl;
    else
      std::cout << "│   │   ├── " << symbole << " false" << std::endl;
  }
}

std::string OntologyReader::getName(const std::string& uri)
{
  size_t pos = uri.find("#");
  std::string result = uri.substr(pos+1);
  return result;
}

inline std::string OntologyReader::getAttribute(TiXmlElement* elem, const std::string& attribute)
{
  const char* subAttr;
  subAttr = elem->Attribute(attribute.c_str());
  if(subAttr != NULL)
    return getName(std::string(subAttr));
  else
    return "";
}

bool OntologyReader::testAttribute(TiXmlElement* subElem, const std::string& attribute)
{
  const char* subAttr;
  subAttr = subElem->Attribute(attribute.c_str());
  if(subAttr != NULL)
    return true;
  else
    return false;
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYREADER_H
