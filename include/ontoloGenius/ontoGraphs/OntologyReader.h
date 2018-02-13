#include "ontoloGenius/ontoGraphs/Ontology.h"
#include "ontoloGenius/utility/utility.h"

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include "ros/ros.h"

#include <tinyxml.h>

#ifndef ONTOLOGY_READER_H
#define ONTOLOGY_READER_H

class OntologyReader
{
public:
  OntologyReader(ClassGraph* p_objTree, PropertyGraph* p_propTree) {m_objTree = p_objTree; m_propTree = p_propTree; elemLoaded = 0; }
  OntologyReader(Ontology& onto) {m_objTree = &onto.classes_; m_propTree = &onto.properties_; elemLoaded = 0; }
  ~OntologyReader() {}

  int readFromUri(std::string uri);
  int readFromFile(std::string fileName);

private:
  ClassGraph* m_objTree;
  PropertyGraph* m_propTree;

  int elemLoaded;

  int read(TiXmlElement* rdf, std::string name);
  void read_class(TiXmlElement* elem);
  void read_individual(TiXmlElement* elem);
  void read_description(TiXmlElement* elem);
  void read_property(TiXmlElement* elem);

  inline void push(std::vector<std::string>& vect, TiXmlElement* subElem, std::string symbole = "", std::string attribute = "rdf:resource");
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

std::string OntologyReader::get_name(std::string uri)
{
  size_t pos = uri.find("#");
  std::string result = uri.substr(pos+1);
  return result;
}

#endif
