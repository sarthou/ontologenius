#include "ontoloGenius/tree.h"
#include "ontoloGenius/utility/utility.h"

#include <vector>
#include <string>
#include <iostream>
#include "ros/ros.h"

#include <tinyxml.h>

#ifndef ONTOLOGY_READER_H
#define ONTOLOGY_READER_H

using namespace std;

enum onto_data_type_t
{
  type_class,
  type_individual
};

class Ontology_reader
{
public:
  Ontology_reader(tree* p_tree) {m_tree = p_tree; }
  ~Ontology_reader() {};

  int read(string uri, onto_data_type_t type);
  int readFile(string fileName, onto_data_type_t type);

private:
  tree* m_tree;

  void read_class(TiXmlElement* elem);
  void read_individual(TiXmlElement* elem);

  string get_name(string uri);
};

#endif
