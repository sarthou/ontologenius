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

using namespace std;

class OntologyReader
{
public:
  OntologyReader(ClassGraph* p_objTree, PropertyGraph* p_propTree) {m_objTree = p_objTree; m_propTree = p_propTree; elemLoaded = 0; }
  ~OntologyReader() {}

  int readFromUri(string uri);
  int readFromFile(string fileName);


private:
  ClassGraph* m_objTree;
  PropertyGraph* m_propTree;

  int elemLoaded;

  int read(TiXmlElement* rdf, string name);
  void read_class(TiXmlElement* elem);
  void read_individual(TiXmlElement* elem);
  void read_description(TiXmlElement* elem);
  void read_property(TiXmlElement* elem);

  void push(vector<string>& vect, TiXmlElement* subElem, string symbole = "", string attribute = "rdf:resource");
  void push(Properties_t& properties, TiXmlElement* subElem, string symbole = "", string attribute = "rdf:resource");
  void pushLang(map<string,string>& dictionary, TiXmlElement* subElem);
  string get_name(string uri);
};

#endif
