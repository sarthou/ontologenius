#include "ontoloGenius/ontology_reader.h"
#include <fstream>
#include "ontoloGenius/utility/error_code.h"

int Ontology_reader::read(string uri, onto_data_type_t type)
{
  int nb_elem = 0;

  string response = "";
  int err = send_request("GET", uri, "", &response);

  if(err == NO_ERROR)
  {
    TiXmlDocument doc;
    doc.Parse((const char*)response.c_str(), 0, TIXML_ENCODING_UTF8);
    TiXmlElement* rdf = doc.FirstChildElement();
    if(rdf == NULL)
    {
        cerr << "Failed to load file: No root element."<< endl;
        return OTHER;
    }
    else
    {
      for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      {
        if(type == type_class)
          read_class(elem);
        else if(type == type_individual)
          read_individual(elem);
      }
      return NO_ERROR;
    }
  }
  else
    return REQUEST_ERROR;
}

int Ontology_reader::readFile(string fileName, onto_data_type_t type)
{
  int nb_elem = 0;

  string response = "";
  string tmp = "";
  std::ifstream f(fileName);
  while(getline(f,tmp))
  {
    response += tmp;
  }

  TiXmlDocument doc;
  doc.Parse((const char*)response.c_str(), 0, TIXML_ENCODING_UTF8);
  TiXmlElement* rdf = doc.FirstChildElement();
  if(rdf == NULL)
  {
      cerr << "Failed to load file: No root element."<< endl;
      return OTHER;
  }
  else
  {
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
    {
      if(type == type_class)
        read_class(elem);
      else if(type == type_individual)
        read_individual(elem);
    }
    return NO_ERROR;
  }
}

void Ontology_reader::read_class(TiXmlElement* elem)
{
  string elemName = elem->Value();
  if(elemName == "owl:Class")
  {
    string node_name = "";
    vector<string> subClass;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      cout << "---" << get_name(string(attr)) << endl;
      node_name = get_name(string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        string subElemName = subElem->Value();
        const char* subAttr;
        if(subElemName == "rdfs:subClassOf")
        {
          subAttr = subElem->Attribute("rdf:resource");
          if(subAttr != NULL)
          {
            subClass.push_back(get_name(string(subAttr)));
            cout << "-" << get_name(string(subAttr)) << endl;
          }
        }
      }
    }
    m_tree->add(node_name, subClass);
    subClass.clear();
  }
}

void Ontology_reader::read_individual(TiXmlElement* elem)
{
  string elemName = elem->Value();
  if(elemName == "owl:NamedIndividual")
  {
    string node_name = "";
    vector<string> subClass;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      cout << "---" << get_name(string(attr)) << endl;
      node_name = get_name(string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        string subElemName = subElem->Value();
        const char* subAttr;
        if(subElemName == "rdf:type")
        {
          subAttr = subElem->Attribute("rdf:resource");
          if(subAttr != NULL)
          {
            subClass.push_back(get_name(string(subAttr)));
            cout << "-" << get_name(string(subAttr)) << endl;
          }
        }
      }
    }
    m_tree->add(node_name, subClass);
    subClass.clear();
  }
}

string Ontology_reader::get_name(string uri)
{
  size_t pos = uri.find("#");
  string result = uri.substr(pos+1);
  return result;
}
