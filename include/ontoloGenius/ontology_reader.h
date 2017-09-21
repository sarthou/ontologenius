#include "ontoloGenius/treeObject.h"
#include "ontoloGenius/treeProperty.h"
#include "ontoloGenius/utility/utility.h"

#include <vector>
#include <string>
#include <iostream>
#include "ros/ros.h"

#include <tinyxml.h>

#ifndef ONTOLOGY_READER_H
#define ONTOLOGY_READER_H

using namespace std;

class Ontology_reader
{
public:
  Ontology_reader(treeObject* p_objTree, treeProperty* p_propTree) {m_objTree = p_objTree; m_propTree = p_propTree; elemLoaded = 0; }
  ~Ontology_reader() {};

  int read(string uri);
  int readFile(string fileName);

private:
  treeObject* m_objTree;
  treeProperty* m_propTree;

  int elemLoaded;

  void read_class(TiXmlElement* elem);
  void read_individual(TiXmlElement* elem);
  void read_description(TiXmlElement* elem);
  void read_property(TiXmlElement* elem);

  string get_name(string uri);
};

#endif
