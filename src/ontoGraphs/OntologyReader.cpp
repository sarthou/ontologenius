#include "ontoloGenius/ontoGraphs/OntologyReader.h"
#include <fstream>
#include "ontoloGenius/utility/error_code.h"

int OntologyReader::readFromUri(string uri)
{
  int nb_elem = 0;

  string response = "";
  int err = send_request("GET", uri, "", &response);

  if(err == NO_ERROR)
  {
    TiXmlDocument doc;
    doc.Parse((const char*)response.c_str(), 0, TIXML_ENCODING_UTF8);
    TiXmlElement* rdf = doc.FirstChildElement();
    return read(rdf, uri);
  }
  else
    return REQUEST_ERROR;
}

int OntologyReader::readFromFile(string fileName)
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
  return read(rdf, fileName);
}

int OntologyReader::read(TiXmlElement* rdf, string name)
{
  if(rdf == NULL)
  {
      cerr << "Failed to load file: No root element."<< endl;
      return OTHER;
  }
  else
  {
    cout << name << endl;
    cout << "************************************" << endl;
    cout << "+ sub       | > domain  | @ language" << endl;
    cout << "- disjoint  | < range   |" << endl;
    cout << "/ inverse   | * type    |" << endl;
    cout << "************************************" << endl;
    cout << "├── Class" << endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      read_class(elem);
    cout << "├── Individuals" << endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      read_individual(elem);
    cout << "├── Description" << endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      read_description(elem);
    cout << "├── Property" << endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      read_property(elem);

    cout << "└── "<< elemLoaded << " readed ! -------------------- " << endl;
    return NO_ERROR;
  }
}

void OntologyReader::read_class(TiXmlElement* elem)
{
  string elemName = elem->Value();
  if(elemName == "owl:Class")
  {
    string node_name = "";
    ObjectVectors_t object_vector;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      cout << "│   ├──" << get_name(string(attr)) << endl;
      node_name = get_name(string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        string subElemName = subElem->Value();

        if(subElemName == "rdfs:subClassOf")
          push(object_vector.mothers_, subElem, "+");
        else if(subElemName == "owl:disjointWith")
          push(object_vector.disjoints_, subElem, "-");
        else if(subElemName == "rdfs:label")
          pushLang(object_vector.dictionary_, subElem);
      }
    }
    m_objTree->add(node_name, object_vector);
    elemLoaded++;
  }
}

void OntologyReader::read_individual(TiXmlElement* elem)
{
  string elemName = elem->Value();
  if(elemName == "owl:NamedIndividual")
  {
    string node_name = "";
    ObjectVectors_t object_vector;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      cout << "│   ├──" << get_name(string(attr)) << endl;
      node_name = get_name(string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        string subElemName = subElem->Value();

        if(subElemName == "rdf:type")
          push(object_vector.mothers_, subElem, "+");
        else if(subElemName == "owl:disjointWith")
          push(object_vector.disjoints_, subElem, "-");
        else if(subElemName == "rdfs:label")
          pushLang(object_vector.dictionary_, subElem);
      }
    }
    m_objTree->add(node_name, object_vector);
    elemLoaded++;
  }
}

void OntologyReader::read_description(TiXmlElement* elem)
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
                  if(subSubElem == subElem->FirstChildElement())
                    cout << "│   ├───┬── -";
                  else if(subSubElem->NextSiblingElement() == NULL)
                    cout << "│   │   └── -";
                  else
                    cout << "│   │   ├── -";
                  cout << get_name(string(subSubAttr)) << endl;
                  disjoints.push_back(get_name(string(subSubAttr)));
                }
              }
            }
          }
        }
      }
    }
    m_objTree->add(disjoints);
    disjoints.clear();
  } // end if(elemName == "rdf:Description")
}

void OntologyReader::read_property(TiXmlElement* elem)
{
  string elemName = elem->Value();
  if(elemName == "owl:ObjectProperty")
  {
    string node_name = "";
    PropertyVectors_t propertyVectors;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      cout << "│   ├──" << get_name(string(attr)) << endl;
      node_name = get_name(string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        string subElemName = subElem->Value();

        if(subElemName == "rdfs:subPropertyOf")
          push(propertyVectors.mothers_, subElem, "+");
        else if(subElemName == "owl:disjointWith")
          push(propertyVectors.disjoints_, subElem, "-");
        else if(subElemName == "owl:inverseOf")
          push(propertyVectors.inverses_, subElem, "/");
        else if(subElemName == "rdfs:domain")
          push(propertyVectors.domains_, subElem, ">");
        else if(subElemName == "rdfs:range")
          push(propertyVectors.ranges_, subElem, "<");
        else if(subElemName == "rdf:type")
          push(propertyVectors.properties_, subElem, "*");
        else if(subElemName == "rdfs:label")
          pushLang(propertyVectors.dictionary_, subElem);
      }
    }

    m_propTree->add(node_name, propertyVectors);
    elemLoaded++;
  }
}

void OntologyReader::push(vector<string>& vect, TiXmlElement* subElem, string symbole, string attribute)
{
  const char* subAttr;
  subAttr = subElem->Attribute(attribute.c_str());
  if(subAttr != NULL)
  {
    vect.push_back(get_name(string(subAttr)));
    cout << "│   │   ├── " << symbole << get_name(string(subAttr)) << endl;
  }
}

void OntologyReader::push(Properties_t& properties, TiXmlElement* subElem, string symbole, string attribute)
{
  const char* subAttr;
  subAttr = subElem->Attribute(attribute.c_str());
  if(subAttr != NULL)
  {
    string property = get_name(string(subAttr));
    if(property == "AsymmetricProperty")
      properties.antisymetric_property_ = true;
    else if(property == "TransitiveProperty")
      properties.transitive_property_ = true; //transitive_with_ ???
    else if(property == "FunctionalProperty")
      properties.functional_property_ = true;
    else if(property == "InverseFunctionalProperty")
      properties.inverse_functional_property_ = true;
    else if(property == "IrreflexiveProperty")
      properties.irreflexive_property_ = true;
    else if(property == "ReflexiveProperty")
      properties.reflexive_property_ = true;
    else if(property == "SymmetricProperty")
      properties.symetric_property_ = true;
    else
      property = "";

    if(property != "")
      cout << "│   │   ├── " << symbole << property << endl;
  }
}

void OntologyReader::pushLang(map<string,string>& dictionary, TiXmlElement* subElem)
{
  const char* subAttr;
  subAttr = subElem->Attribute("xml:lang");
  if(subAttr != NULL)
  {
    string lang = get_name(string(subAttr));

    const char* value;
    value = subElem->GetText();
    dictionary[lang] = string(value);

    if((lang != "") && (value != ""))
      cout << "│   │   ├── " << "@" << lang << " : " << dictionary[lang] << endl;
  }
}

string OntologyReader::get_name(string uri)
{
  size_t pos = uri.find("#");
  string result = uri.substr(pos+1);
  return result;
}
