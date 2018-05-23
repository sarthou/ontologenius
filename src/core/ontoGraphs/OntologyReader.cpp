#include "ontoloGenius/core/ontoGraphs/OntologyReader.h"
#include <fstream>
#include "ontoloGenius/core/utility/error_code.h"
#include "ontoloGenius/core/ontoGraphs/Ontology.h"

#include "ontoloGenius/core/utility/utility.h"

OntologyReader::OntologyReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph)
{
  class_graph_ = class_graph;
  object_property_graph_ = object_property_graph;
  data_property_graph_ = data_property_graph;
  individual_graph_ = individual_graph;
  elemLoaded = 0;
}

OntologyReader::OntologyReader(Ontology& onto)
{
  class_graph_ = &onto.class_graph_;
  object_property_graph_ = &onto.object_property_graph_;
  individual_graph_ = &onto.individual_graph_;
  data_property_graph_ = &onto.data_property_graph_;
  elemLoaded = 0;
}


int OntologyReader::readFromUri(std::string uri, bool individual)
{
  std::string response = "";
  int err = send_request("GET", uri, "", &response);

  if(err == NO_ERROR)
  {
    TiXmlDocument doc;
    doc.Parse((const char*)response.c_str(), 0, TIXML_ENCODING_UTF8);
    TiXmlElement* rdf = doc.FirstChildElement();
    if(individual == false)
      return read(rdf, uri);
    else
      return readIndividual(rdf, uri);
  }
  else
    return REQUEST_ERROR;
}

int OntologyReader::readFromFile(std::string fileName, bool individual)
{
  std::string response = "";
  std::string tmp = "";
  std::ifstream f(fileName);
  while(getline(f,tmp))
  {
    response += tmp;
  }

  TiXmlDocument doc;
  doc.Parse((const char*)response.c_str(), 0, TIXML_ENCODING_UTF8);
  TiXmlElement* rdf = doc.FirstChildElement();
  if(individual == false)
    return read(rdf, fileName);
  else
    return readIndividual(rdf, fileName);
}

int OntologyReader::read(TiXmlElement* rdf, std::string name)
{
  if(rdf == NULL)
  {
      std::cerr << "Failed to load file: No root element."<< std::endl;
      return OTHER;
  }
  else
  {
    std::cout << name << std::endl;
    std::cout << "************************************" << std::endl;
    std::cout << "+ sub       | > domain  | @ language" << std::endl;
    std::cout << "- disjoint  | < range   | . chain axiom" << std::endl;
    std::cout << "/ inverse   | * type    | " << std::endl;
    std::cout << "************************************" << std::endl;
    std::cout << "├── Class" << std::endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      readClass(elem);
    std::cout << "├── Description" << std::endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      readDescription(elem);
    std::cout << "├── Object property" << std::endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      readObjectProperty(elem);
    std::cout << "├── Data property" << std::endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      readDataProperty(elem);

    std::cout << "└── "<< elemLoaded << " readed ! " << std::endl;
    return NO_ERROR;
  }
}

void OntologyReader::displayIndividualRules()
{
  std::cout << "************************************" << std::endl;
  std::cout << "+ sub        | = same       | - distinct" << std::endl;
  std::cout << "^ related    | # data type  | @ language" << std::endl;
  std::cout << "************************************" << std::endl;
}

int OntologyReader::readIndividual(TiXmlElement* rdf, std::string name)
{
  if(rdf == NULL)
  {
      std::cerr << "Failed to load file: No root element."<< std::endl;
      return OTHER;
  }
  else
  {
    std::cout << name << std::endl;
    std::cout << "├── Individuals" << std::endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      readIndividual(elem);
    std::cout << "├── Description" << std::endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
      readIndividualDescription(elem);

    std::cout << "└── "<< elemLoaded << " readed ! " << std::endl;
    return NO_ERROR;
  }
}

void OntologyReader::readClass(TiXmlElement* elem)
{
  std::string elemName = elem->Value();
  if(elemName == "owl:Class")
  {
    std::string node_name = "";
    ObjectVectors_t object_vector;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      std::cout << "│   ├──" << getName(std::string(attr)) << std::endl;
      node_name = getName(std::string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        std::string subElemName = subElem->Value();

        if(subElemName == "rdfs:subClassOf")
          push(object_vector.mothers_, subElem, "+");
        else if(subElemName == "owl:disjointWith")
          push(object_vector.disjoints_, subElem, "-");
        else if(subElemName == "rdfs:label")
          pushLang(object_vector.dictionary_, subElem);
      }
    }
    class_graph_->add(node_name, object_vector);
    elemLoaded++;
  }
}

void OntologyReader::readIndividual(TiXmlElement* elem)
{
  std::string elemName = elem->Value();
  if(elemName == "owl:NamedIndividual")
  {
    std::string node_name = "";
    IndividualVectors_t individual_vector;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      std::cout << "│   ├──" << getName(std::string(attr)) << std::endl;
      node_name = getName(std::string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        std::string subElemName = subElem->Value();

        if(subElemName == "rdf:type")
          push(individual_vector.is_a_, subElem, "+");
        else if(subElemName == "owl:sameAs")
          push(individual_vector.same_as_, subElem, "=");
        else if(subElemName == "rdfs:label")
          pushLang(individual_vector.dictionary_, subElem);
        else
        {
          std::string ns = subElemName.substr(0,subElemName.find(":"));
          if((ns != "owl") && (ns != "rdf") && (ns != "rdfs"))
          {
            std::string property = subElemName.substr(subElemName.find(":")+1);
            if(testAttribute(subElem, "rdf:resource"))
            {
              push(individual_vector.object_properties_name_, property, "+");
              push(individual_vector.object_properties_on_, subElem, "^");
            }
            else if(testAttribute(subElem, "rdf:datatype"))
            {
              push(individual_vector.data_properties_name_, property, "+");
              const char* value = subElem->GetText();
              push(individual_vector.data_properties_value_, std::string(value), "^");
              push(individual_vector.data_properties_type_, subElem, "#", "rdf:datatype");
            }
          }
        }
      }
    }
    individual_graph_->add(node_name, individual_vector);
    elemLoaded++;
  }
}

void OntologyReader::readDescription(TiXmlElement* elem)
{
  std::string elemName = elem->Value();
  if(elemName == "rdf:Description")
  {
    std::vector<std::string> disjoints;
    bool isAllDisjointClasses = false;
    bool isAllDisjointProperties = false;

    for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
    {
      std::string subElemName = subElem->Value();
      const char* subAttr;
      if(subElemName == "rdf:type")
      {
        subAttr = subElem->Attribute("rdf:resource");
        if(subAttr != NULL)
        {
          if(getName(std::string(subAttr)) == "AllDisjointClasses")
            isAllDisjointClasses = true;
          else if(getName(std::string(subAttr)) == "AllDisjointProperties")
            isAllDisjointProperties = true;
        }
      }
      else if(subElemName == "owl:members")
      {
        subAttr = subElem->Attribute("rdf:parseType");
        if(subAttr != NULL)
          if(std::string(subAttr) == "Collection")
            readCollection(disjoints, subElem, "-");
      }
    }

    if(isAllDisjointClasses)
      class_graph_->add(disjoints);
    else if(isAllDisjointProperties)
      object_property_graph_->add(disjoints);
    disjoints.clear();
  } // end if(elemName == "rdf:Description")
}

void OntologyReader::readIndividualDescription(TiXmlElement* elem)
{
  std::string elemName = elem->Value();
  if(elemName == "rdf:Description")
  {
    std::vector<std::string> distincts;
    bool isDistincttAll = false;
    for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
    {
      std::string subElemName = subElem->Value();
      const char* subAttr;
      if(subElemName == "rdf:type")
      {
        subAttr = subElem->Attribute("rdf:resource");
        if(subAttr != NULL)
          if(getName(std::string(subAttr)) == "AllDifferent")
            isDistincttAll = true;
      }
      else if(subElemName == "owl:distinctMembers")
      {
        subAttr = subElem->Attribute("rdf:parseType");
        if(subAttr != NULL)
          if(std::string(subAttr) == "Collection")
            readCollection(distincts, subElem, "-");
      }
    }
    if(isDistincttAll)
      individual_graph_->add(distincts);
    distincts.clear();
  } // end if(elemName == "rdf:Description")
}

void OntologyReader::readObjectProperty(TiXmlElement* elem)
{
  std::string elemName = elem->Value();
  if(elemName == "owl:ObjectProperty")
  {
    std::string node_name = "";
    ObjectPropertyVectors_t propertyVectors;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      std::cout << "│   ├──" << getName(std::string(attr)) << std::endl;
      node_name = getName(std::string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        std::string subElemName = subElem->Value();

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
        else if(subElemName == "owl:propertyChainAxiom")
        {
          std::vector<std::string> tmp;
          readCollection(tmp, subElem, ".", 2);
          propertyVectors.chains_.push_back(tmp);
        }
      }
    }

    object_property_graph_->add(node_name, propertyVectors);
    elemLoaded++;
  }
}

void OntologyReader::readDataProperty(TiXmlElement* elem)
{
  std::string elemName = elem->Value();
  if(elemName == "owl:DatatypeProperty")
  {
    std::string node_name = "";
    DataPropertyVectors_t propertyVectors;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != NULL)
    {
      std::cout << "│   ├──" << getName(std::string(attr)) << std::endl;
      node_name = getName(std::string(attr));
      for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
      {
        std::string subElemName = subElem->Value();

        if(subElemName == "rdfs:subPropertyOf")
          push(propertyVectors.mothers_, subElem, "+");
        else if(subElemName == "owl:disjointWith")
          push(propertyVectors.disjoints_, subElem, "-");
        else if(subElemName == "rdfs:domain")
          push(propertyVectors.domains_, subElem, ">");
        else if(subElemName == "rdfs:range")
          push(propertyVectors.ranges_, subElem, "<");
        else if(subElemName == "rdfs:label")
          pushLang(propertyVectors.dictionary_, subElem);
      }
    }

    data_property_graph_->add(node_name, propertyVectors);
    elemLoaded++;
  }
}

void OntologyReader::readCollection(std::vector<std::string>& vect, TiXmlElement* elem, std::string symbol, size_t level)
{
  for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
  {
    std::string subElemName = subElem->Value();
    const char* subAttr;
    if(subElemName == "rdf:Description")
    {
      subAttr = subElem->Attribute("rdf:about");
      if(subAttr != NULL)
      {
        for(size_t i = 0; i < level; i++)
          std::cout << "│   ";
        if(subElem == elem->FirstChildElement())
          std::cout << "├───┬── " << symbol;
        else if(subElem->NextSiblingElement() == NULL)
          std::cout << "│   └── " << symbol;
        else
          std::cout << "│   ├── " << symbol;
        std::cout << getName(std::string(subAttr)) << std::endl;
        vect.push_back(getName(std::string(subAttr)));
      }
    }
  }
}

void OntologyReader::readRestriction(TiXmlElement* elem)
{
  std::string on;
  std::vector<std::string> restriction;

  for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
  {
    std::string restriction_name = subElem->Value();
    if(restriction_name == "owl:onProperty")
      on = getAttribute(subElem, "rdf:resource");
    else if(restriction_name == "owl:someValuesFrom")
      restriction.push_back(readSomeValuesFrom(subElem));
  }
  //std::cout << "on " << on << std::endl;
}

std::string OntologyReader::readSomeValuesFrom(TiXmlElement* elem)
{
  std::string value = getAttribute(elem, "rdf:resource");
  if(value == "")
    for(TiXmlElement* subElem = elem->FirstChildElement(); subElem != NULL; subElem = subElem->NextSiblingElement())
    {
      std::string restriction_name = subElem->Value();
      if(restriction_name == "rdfs:Datatype")
        std::cout << restriction_name << std::endl;
    }
  return value;
}

void OntologyReader::push(Properties_t& properties, TiXmlElement* subElem, std::string symbole, std::string attribute)
{
  const char* subAttr;
  subAttr = subElem->Attribute(attribute.c_str());
  if(subAttr != NULL)
  {
    std::string property = getName(std::string(subAttr));
    if(property == "AsymmetricProperty")
      properties.antisymetric_property_ = true;
    else if(property == "TransitiveProperty")
      properties.transitive_property_ = true;
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
      std::cout << "│   │   ├── " << symbole << property << std::endl;
  }
}

void OntologyReader::pushLang(std::map<std::string, std::vector<std::string>>& dictionary, TiXmlElement* subElem)
{
  const char* subAttr;
  subAttr = subElem->Attribute("xml:lang");
  if(subAttr != NULL)
  {
    std::string lang = getName(std::string(subAttr));

    const char* value;
    value = subElem->GetText();
    dictionary[lang].push_back(std::string(value));

    if((lang != "") && (std::string(value) != ""))
      std::cout << "│   │   ├── " << "@" << lang << " : " << dictionary[lang][dictionary[lang].size() - 1] << std::endl;
  }
}
