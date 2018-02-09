#include "ontoloGenius/ontoGraphs/ClassGraph.h"
#include "ontoloGenius/ontoGraphs/PropertyGraph.h"
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

  void push(std::vector<std::string>& vect, TiXmlElement* subElem, std::string symbole = "", std::string attribute = "rdf:resource");
  void push(Properties_t& properties, TiXmlElement* subElem, std::string symbole = "", std::string attribute = "rdf:resource");
  void pushLang(std::map<std::string, std::string>& dictionary, TiXmlElement* subElem);
  std::string get_name(std::string uri);
};

#endif
