#include "ontoloGenius/ontology_reader.h"
#include <fstream>
#include "ontoloGenius/utility/error_code.h"

int Ontology_reader::read(string uri)
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
        read_class(elem);
        read_individual(elem);
        read_description(elem);
      }
      return NO_ERROR;
    }
  }
  else
    return REQUEST_ERROR;
}

int Ontology_reader::readFile(string fileName)
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
      read_class(elem);
      read_individual(elem);
      read_description(elem);
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
    vector<string> disjoint;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      cout << "-->" << get_name(string(attr)) << endl;
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
            cout << "+" << get_name(string(subAttr)) << endl;
          }
        }
        else if(subElemName == "owl:disjointWith")
        {
          subAttr = subElem->Attribute("rdf:resource");
          if(subAttr != NULL)
          {
            disjoint.push_back(get_name(string(subAttr)));
            cout << "-" << get_name(string(subAttr)) << endl;
          }
        }
      }
    }
    m_tree->add(node_name, subClass, disjoint);
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
    vector<string> disjoint;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      cout << "-->" << get_name(string(attr)) << endl;
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
            cout << "+" << get_name(string(subAttr)) << endl;
          }
        }
        else if(subElemName == "owl:disjointWith")
        {
          subAttr = subElem->Attribute("rdf:resource");
          if(subAttr != NULL)
          {
            disjoint.push_back(get_name(string(subAttr)));
            cout << "-" << get_name(string(subAttr)) << endl;
          }
        }
      }
    }
    m_tree->add(node_name, subClass, disjoint);
    subClass.clear();
  }
}

void Ontology_reader::read_description(TiXmlElement* elem)
{
  string elemName = elem->Value();
  if(elemName == "rdf:Description")
  {
    vector<string> disjoints;
    bool isDisjointAll = false;
    for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
    {
      string subElemName = subElem->Value();
      const char* subAttr;
      if(subElemName == "rdf:type")
      {
        subAttr = subElem->Attribute("rdf:resource");
        if(subAttr != NULL)
        {
          if(get_name(string(subAttr)) == "AllDisjointClasses")
            isDisjointAll = true;
        }
      }
      else if(subElemName == "owl:members")
      {
        subAttr = subElem->Attribute("rdf:parseType");
        if(subAttr != NULL)
        {
          if(string(subAttr) == "Collection")
          {
            for(TiXmlElement* subSubElem = subElem->FirstChildElement(); subSubElem != NULL; subSubElem = subSubElem->NextSiblingElement())
            {
              string subSubElemName = subSubElem->Value();
              const char* subSubAttr;
              if(subSubElemName == "rdf:Description")
              {
                subSubAttr = subSubElem->Attribute("rdf:about");
                if(subSubAttr != NULL)
                {
                  cout << "<--" << get_name(string(subSubAttr)) << endl;
                  disjoints.push_back(get_name(string(subSubAttr)));
                }
              }
            }
          }
        }
      }
    }
    m_tree->add(disjoints);
    disjoints.clear();
  } // end if(elemName == "rdf:Description")
}

string Ontology_reader::get_name(string uri)
{
  size_t pos = uri.find("#");
  string result = uri.substr(pos+1);
  return result;
}
