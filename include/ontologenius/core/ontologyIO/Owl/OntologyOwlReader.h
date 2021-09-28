#ifndef ONTOLOGENIUS_ONTOLOGYOWLREADER_H
#define ONTOLOGENIUS_ONTOLOGYOWLREADER_H

#include <vector>
#include <string>
#include <map>
#include <iostream>

#include <tinyxml.h>

#include <ros/ros.h>

#include "ontologenius/core/ontologyIO/OntologyReader.h"

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

class Ontology;

class OntologyOwlReader : public OntologyReader
{
public:
  OntologyOwlReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph) :
                    OntologyReader(class_graph, object_property_graph, data_property_graph, individual_graph) {}
  OntologyOwlReader(Ontology& onto) : OntologyReader(onto) {}
  ~OntologyOwlReader() {}

  int readFromUri(const std::string& uri, bool individual = false);
  int readFromFile(const std::string& fileName, bool individual = false);

  void setDisplay(bool display) { display_ = display; }
  void displayIndividualRules();
  bool empty() {return (elemLoaded == 0); }

private:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;
  IndividualGraph* individual_graph_;

  int elemLoaded;
  bool display_;

  int read(TiXmlElement* rdf, const std::string& name);
  int readIndividual(TiXmlElement* rdf, const std::string& name);

  void readClass(TiXmlElement* elem);
  void readIndividual(TiXmlElement* elem);
  void readDescription(TiXmlElement* elem);
  void readIndividualDescription(TiXmlElement* elem);
  void readObjectProperty(TiXmlElement* elem);
  void readDataProperty(TiXmlElement* elem);
  void readAnnotationProperty(TiXmlElement* elem);
  void readCollection(std::vector<std::string>& vect, TiXmlElement* elem, const std::string& symbol, size_t level = 1);
  std::string readSomeValuesFrom(TiXmlElement* elem);

  inline void push(std::vector<std::string>& vect, TiXmlElement* subElem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
  inline void push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole = "");
  inline void push(std::vector<Single_t<std::string>>& vect, TiXmlElement* subElem, float probability, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
  inline void push(std::vector<Pair_t<std::string, std::string>>& vect, const Pair_t<std::string, std::string>& elem, const std::string& symbole1, const std::string& symbole2);
  inline void push(std::vector<Pair_t<std::string, data_t>>& vect, const Pair_t<std::string, data_t>& elem, const std::string& symbole1, const std::string& symbole2);
  inline void push(std::vector<bool>& vect, bool elem, const std::string& symbole = "");
  void push(Properties_t& properties, TiXmlElement* subElem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
  void pushLang(std::map<std::string, std::vector<std::string>>& dictionary, TiXmlElement* subElem);
  inline std::string getName(const std::string& uri);
  inline float getProbability(TiXmlElement* elem);
  inline std::string getAttribute(TiXmlElement* elem, const std::string& attribute);
  inline bool testAttribute(TiXmlElement* subElem, const std::string& attribute);

  std::string toString(TiXmlElement* subElem, std::string attribute = "rdf:resource")
  {
    const char* subAttr;
    subAttr = subElem->Attribute(attribute.c_str());
    if(subAttr != NULL)
      return getName(std::string(subAttr));
    return "";
  }

  void removeDocType(std::string& txt);
};

void OntologyOwlReader::push(std::vector<std::string>& vect, TiXmlElement* subElem, const std::string& symbole, const std::string& attribute)
{
  std::string data = getAttribute(subElem, attribute);
  if(data != "")
  {
    vect.push_back(data);
    if(symbole != "" && display_)
      std::cout << "│   │   ├── " << symbole << data << std::endl;
  }
}

void OntologyOwlReader::push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole)
{
  vect.push_back(elem);
  if(symbole != "" && display_)
    std::cout << "│   │   ├── " << symbole << elem << std::endl;
}

void OntologyOwlReader::push(std::vector<Single_t<std::string>>& vect, TiXmlElement* subElem, float probability, const std::string& symbole, const std::string& attribute)
{
  std::string data = getAttribute(subElem, attribute);
  if(data != "")
  {
    vect.push_back(Single_t<std::string>(data, probability));
    if(symbole != "" && display_)
      std::cout << "│   │   ├── " << symbole << data << std::endl;
  }
}

void OntologyOwlReader::push(std::vector<Pair_t<std::string, std::string>>& vect, const Pair_t<std::string, std::string>& elem, const std::string& symbole1, const std::string& symbole2)
{
  vect.push_back(elem);
  if(symbole1 != "" && display_)
    std::cout << "│   │   ├── " << symbole1 << elem.first << std::endl;

  if(symbole2 != "" && display_)
    std::cout << "│   │   ├── " << symbole2 << elem.second << std::endl;
}

void OntologyOwlReader::push(std::vector<Pair_t<std::string, data_t>>& vect, const Pair_t<std::string, data_t>& elem, const std::string& symbole1, const std::string& symbole2)
{
  vect.push_back(elem);
  if(symbole1 != "" && display_)
    std::cout << "│   │   ├── " << symbole1 << elem.first << std::endl;

  if(symbole2 != "" && display_)
    std::cout << "│   │   ├── " << symbole2 << elem.second.toString() << std::endl;
}

void OntologyOwlReader::push(std::vector<bool>& vect, bool elem, const std::string& symbole)
{
  vect.push_back(elem);
  if(symbole != "" && display_)
  {
    if(elem == true)
      std::cout << "│   │   ├── " << symbole << " true" << std::endl;
    else
      std::cout << "│   │   ├── " << symbole << " false" << std::endl;
  }
}

std::string OntologyOwlReader::getName(const std::string& uri)
{
  size_t pos = uri.find("#");
  std::string result = uri.substr(pos+1);
  return result;
}

float OntologyOwlReader::getProbability(TiXmlElement* elem)
{
  float proba = 1.0;

  const char* subAttr;
  subAttr = elem->Attribute("onto:probability");
  if(subAttr != NULL)
    proba = std::stof(std::string(subAttr));

  return proba;
}

inline std::string OntologyOwlReader::getAttribute(TiXmlElement* elem, const std::string& attribute)
{
  const char* subAttr;
  subAttr = elem->Attribute(attribute.c_str());
  if(subAttr != NULL)
    return getName(std::string(subAttr));
  else
    return "";
}

bool OntologyOwlReader::testAttribute(TiXmlElement* subElem, const std::string& attribute)
{
  const char* subAttr;
  subAttr = subElem->Attribute(attribute.c_str());
  if(subAttr != NULL)
    return true;
  else
    return false;
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYOWLREADER_H
