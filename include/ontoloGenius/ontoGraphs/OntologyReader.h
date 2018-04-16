#ifndef ONTOLOGY_READER_H
#define ONTOLOGY_READER_H

#include "ontoloGenius/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/PropertyGraph.h"
#include "ontoloGenius/ontoGraphs/Graphs/IndividualGraph.h"

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include "ros/ros.h"

#include <tinyxml.h>

class Ontology;

class OntologyReader
{
public:
  OntologyReader(ClassGraph* p_objTree, PropertyGraph* p_propTree, IndividualGraph* individual_graph);
  OntologyReader(Ontology& onto);
  ~OntologyReader() {}

  int readFromUri(std::string uri, bool individual = false);
  int readFromFile(std::string fileName, bool individual = false);

  void displayIndividualRules();

private:
  ClassGraph* m_objTree;
  PropertyGraph* m_propTree;
  IndividualGraph* individual_graph_;

  int elemLoaded;

  int read(TiXmlElement* rdf, std::string name);
  int readIndividual(TiXmlElement* rdf, std::string name);

  void read_class(TiXmlElement* elem);
  void read_individual(TiXmlElement* elem);
  void read_description(TiXmlElement* elem);
  void read_individual_description(TiXmlElement* elem);
  void read_property(TiXmlElement* elem);
  void readCollection(std::vector<std::string>& vect, TiXmlElement* elem, std::string symbol);

  inline void push(std::vector<std::string>& vect, TiXmlElement* subElem, std::string symbole = "", std::string attribute = "rdf:resource");
  inline void push(std::vector<std::string>& vect, std::string elem, std::string symbole);
  void push(Properties_t& properties, TiXmlElement* subElem, std::string symbole = "", std::string attribute = "rdf:resource");
  void pushLang(std::map<std::string, std::string>& dictionary, TiXmlElement* subElem);
  inline std::string get_name(std::string uri);
};

void OntologyReader::push(std::vector<std::string>& vect, TiXmlElement* subElem, std::string symbole, std::string attribute)
{
  const char* subAttr;
  subAttr = subElem->Attribute(attribute.c_str());
  if(subAttr != NULL)
  {
    vect.push_back(get_name(std::string(subAttr)));
    std::cout << "│   │   ├── " << symbole << get_name(std::string(subAttr)) << std::endl;
  }
}

void OntologyReader::push(std::vector<std::string>& vect, std::string elem, std::string symbole)
{
  vect.push_back(elem);
  std::cout << "│   │   ├── " << symbole << elem << std::endl;
}

std::string OntologyReader::get_name(std::string uri)
{
  size_t pos = uri.find("#");
  std::string result = uri.substr(pos+1);
  return result;
}

#endif
