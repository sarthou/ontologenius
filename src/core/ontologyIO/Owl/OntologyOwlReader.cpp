#include "ontologenius/core/ontologyIO/Owl/OntologyOwlReader.h"

#include <cstddef>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <tinyxml2.h>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/OntologyGraphs.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"
#include "ontologenius/core/ontologyIO/OntologyReader.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

  OntologyOwlReader::OntologyOwlReader(OntologyGraphs* graphs) : OntologyReader(graphs),
                                                                 card_map_{
                                                                   {"owl:someValuesFrom",          "some"   },
                                                                   {"owl:allValuesFrom",           "only"   },
                                                                   {"owl:minQualifiedCardinality", "min"    },
                                                                   {"owl:maxQualifiedCardinality", "max"    },
                                                                   {"owl:qualifiedCardinality",    "exactly"},
                                                                   {"owl:hasValue",                "value"  }
  }
  {}

  int OntologyOwlReader::readFromUri(std::string content, const std::string& uri, bool individual)
  {
    removeDocType(content);

    tinyxml2::XMLDocument doc;
    doc.Parse(static_cast<const char*>(content.c_str()));
    tinyxml2::XMLElement* rdf = doc.FirstChildElement();
    if(individual == false)
      return read(rdf, uri);
    else
      return readIndividual(rdf, uri);
  }

  int OntologyOwlReader::readFromFile(const std::string& file_name, bool individual)
  {
    std::string response;
    std::string tmp;
    std::ifstream f(file_name);

    if(!f.is_open())
    {
      Display::error("Fail to open : " + file_name);
      return -1;
    }

    while(getline(f, tmp))
    {
      response += tmp;
    }
    removeDocType(response);

    tinyxml2::XMLDocument doc;
    doc.Parse(static_cast<const char*>(response.c_str()));
    tinyxml2::XMLElement* rdf = doc.FirstChildElement();
    if(individual == false)
      return read(rdf, file_name);
    else
      return readIndividual(rdf, file_name);
  }

  std::vector<std::string> OntologyOwlReader::getImportsFromRaw(std::string content)
  {
    std::vector<std::string> imports;
    removeDocType(content);

    tinyxml2::XMLDocument doc;
    doc.Parse(static_cast<const char*>(content.c_str()));
    tinyxml2::XMLElement* rdf = doc.FirstChildElement();

    if(rdf == nullptr)
      return {};
    else if(std::string(rdf->Value()) != "rdf:RDF")
      return {};
    else
    {
      auto* ontology_elem = rdf->FirstChildElement("owl:Ontology");
      for(tinyxml2::XMLElement* elem = ontology_elem->FirstChildElement("owl:imports"); elem != nullptr; elem = elem->NextSiblingElement("owl:imports"))
        imports.emplace_back(elem->Attribute("rdf:resource"));
    }

    return imports;
  }

  std::vector<std::string> OntologyOwlReader::getImportsFromFile(const std::string& file_name)
  {
    std::string raw_file;
    std::string tmp;
    std::ifstream f(file_name);

    if(!f.is_open())
      return {};

    while(getline(f, tmp))
      raw_file += tmp;

    return getImportsFromRaw(raw_file);
  }

  int OntologyOwlReader::read(tinyxml2::XMLElement* rdf, const std::string& name)
  {
    if(rdf == nullptr)
    {
      Display::error("Failed to read file: " + name);
      return OTHER;
    }
    else if(std::string(rdf->Value()) != "rdf:RDF")
    {
      Display::error("File is not based on RDF: " + name);
      return OTHER;
    }
    else
    {
      if(display_)
      {
        std::cout << name << std::endl;
        std::cout << "************************************" << std::endl;
        std::cout << "+ sub          | > domain  | @ language" << std::endl;
        std::cout << "- disjoint     | < range   | . chain axiom" << std::endl;
        std::cout << "/ inverse      | * type    | = equivalence" << std::endl;
        std::cout << "$ has property | ^ related | " << std::endl;
        std::cout << "************************************" << std::endl;
      }

      if(display_)
        std::cout << "├── Object property" << std::endl;
      for(tinyxml2::XMLElement* elem = rdf->FirstChildElement("owl:ObjectProperty"); elem != nullptr; elem = elem->NextSiblingElement("owl:ObjectProperty"))
        readObjectProperty(elem);
      if(display_)
        std::cout << "├── Data property" << std::endl;
      for(tinyxml2::XMLElement* elem = rdf->FirstChildElement("owl:DatatypeProperty"); elem != nullptr; elem = elem->NextSiblingElement("owl:DatatypeProperty"))
        readDataProperty(elem);
      if(display_)
        std::cout << "├── Class" << std::endl;
      for(tinyxml2::XMLElement* elem = rdf->FirstChildElement("owl:Class"); elem != nullptr; elem = elem->NextSiblingElement("owl:Class"))
        readClass(elem);
      if(display_)
        std::cout << "├── Description" << std::endl;
      for(tinyxml2::XMLElement* elem = rdf->FirstChildElement("rdf:Description"); elem != nullptr; elem = elem->NextSiblingElement("rdf:Description"))
        readDescription(elem);
      if(display_)
        std::cout << "├── Annotation property" << std::endl;
      for(tinyxml2::XMLElement* elem = rdf->FirstChildElement("owl:AnnotationProperty"); elem != nullptr; elem = elem->NextSiblingElement("owl:AnnotationProperty"))
        readAnnotationProperty(elem);
      if(display_)
        std::cout << "└── " << nb_loaded_elem_ << " readed ! " << std::endl;
      return NO_ERROR;
    }
  }

  void OntologyOwlReader::displayIndividualRules()
  {
    if(display_)
    {
      std::cout << "************************************" << std::endl;
      std::cout << "+ is a         | = same       | - distinct" << std::endl;
      std::cout << "$ has property | ^ related    | @ language" << std::endl;
      std::cout << "************************************" << std::endl;
    }
  }

  int OntologyOwlReader::readIndividual(tinyxml2::XMLElement* rdf, const std::string& name)
  {
    if(rdf == nullptr)
    {
      Display::error("Failed to load file: " + name + "\n\t- No root element.");
      return OTHER;
    }
    else
    {
      if(display_)
      {
        std::cout << name << std::endl;
        std::cout << "├── Individuals" << std::endl;
      }
      for(tinyxml2::XMLElement* elem = rdf->FirstChildElement("owl:NamedIndividual"); elem != nullptr; elem = elem->NextSiblingElement("owl:NamedIndividual"))
        readIndividual(elem);
      if(display_)
        std::cout << "├── Description" << std::endl;
      for(tinyxml2::XMLElement* elem = rdf->FirstChildElement("rdf:Description"); elem != nullptr; elem = elem->NextSiblingElement("rdf:Description"))
        readIndividualDescription(elem);
      if(display_)
        std::cout << "└── " << nb_loaded_elem_ << " readed ! " << std::endl;
      return NO_ERROR;
    }
  }

  void OntologyOwlReader::readClass(tinyxml2::XMLElement* elem)
  {
    ClassDescriptor_t class_descriptor;
    EquivalentClassDescriptor_t anonymous_descriptor;
    EquivalentClassDescriptor_t sub_anonymous_descriptor;
    std::string node_name = getAttribute(elem, std::vector<std::string>{"rdf:about", "rdf:ID"});
    if(node_name.empty() == false)
    {
      if(display_)
        std::cout << "│   ├──" << node_name << std::endl;
      for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      {
        const std::string sub_elem_name = sub_elem->Value();

        const float probability = getProbability(sub_elem);

        if(sub_elem_name == "rdfs:subClassOf")
        {
          // Distinguish complex (anonymous) from simple (named) subClassOf
          if(sub_elem->FirstChildElement() != nullptr && !testAttribute(sub_elem, "rdf:resource"))
            sub_anonymous_descriptor.expression_members.emplace_back(readEquivalentClass(sub_elem));
          else
            push(class_descriptor.mothers_, sub_elem, probability, "+");
        }
        else if(sub_elem_name == "owl:disjointWith")
          push(class_descriptor.disjoints_, sub_elem, probability, "-");
        else if(sub_elem_name == "rdfs:label")
          pushLang(class_descriptor.dictionary_, sub_elem);
        else if(sub_elem_name == "onto:label")
          pushLang(class_descriptor.muted_dictionary_, sub_elem);
        else if(sub_elem_name == "rdfs:comment")
          pushComment(class_descriptor.comments_, sub_elem);
        else if(sub_elem_name == "owl:equivalentClass")
          anonymous_descriptor.expression_members.emplace_back(readEquivalentClass(sub_elem));
        else
        {
          const std::string ns = sub_elem_name.substr(0, sub_elem_name.find(':'));
          if((ns != "owl") && (ns != "rdf") && (ns != "rdfs"))
          {
            const std::string property = sub_elem_name.substr(sub_elem_name.find(':') + 1);
            if(testAttribute(sub_elem, "rdf:resource"))
              OntologyReader::push(class_descriptor.object_relations_, PairElement<std::string, std::string>(property, toString(sub_elem), probability), "$", "^");
            else if(testAttribute(sub_elem, "rdf:datatype"))
            {
              const char* value = sub_elem->GetText();
              if(value != nullptr)
                OntologyReader::push(class_descriptor.data_relations_, PairElement<std::string, std::string>(property, toString(sub_elem, "rdf:datatype") + "#" + std::string(value), probability), "$", "^");
            }
          }
        }
      }
    }

    if(anonymous_descriptor.expression_members.empty() == false)
    {
      anonymous_descriptor.class_name = node_name;
      graphs_->anonymous_classes_.add(anonymous_descriptor);
      for(auto* expression_elem : anonymous_descriptor.expression_members)
        display(expression_elem->toString(), "=");
    }

    if(sub_anonymous_descriptor.expression_members.empty() == false)
    {
      sub_anonymous_descriptor.class_name = node_name;
      graphs_->anonymous_classes_.addSubClass(sub_anonymous_descriptor);
      for(auto* expression_elem : sub_anonymous_descriptor.expression_members)
        display(expression_elem->toString(), "+");
    }

    graphs_->classes_.add(node_name, class_descriptor);
    nb_loaded_elem_++;
  }

  void OntologyOwlReader::readIndividual(tinyxml2::XMLElement* elem)
  {
    IndividualDescriptor_t individual_descriptor;
    std::string node_name = getAttribute(elem, std::vector<std::string>{"rdf:about", "rdf:ID"});
    if(node_name.empty() == false)
    {
      if(display_)
        std::cout << "│   ├──" << node_name << std::endl;
      for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      {
        const std::string sub_elem_name = sub_elem->Value();
        const float probability = getProbability(sub_elem);

        if(sub_elem_name == "rdf:type")
          push(individual_descriptor.is_a_, sub_elem, probability, "+");
        else if(sub_elem_name == "owl:sameAs")
          push(individual_descriptor.same_as_, sub_elem, probability, "=");
        else if(sub_elem_name == "rdfs:label")
          pushLang(individual_descriptor.dictionary_, sub_elem);
        else if(sub_elem_name == "onto:label")
          pushLang(individual_descriptor.muted_dictionary_, sub_elem);
        else if(sub_elem_name == "rdfs:comment")
          pushComment(individual_descriptor.comments_, sub_elem);
        else
        {
          const std::string ns = sub_elem_name.substr(0, sub_elem_name.find(':'));
          if((ns != "owl") && (ns != "rdf") && (ns != "rdfs"))
          {
            const std::string property = sub_elem_name.substr(sub_elem_name.find(':') + 1);
            if(testAttribute(sub_elem, "rdf:resource"))
              OntologyReader::push(individual_descriptor.object_relations_, PairElement<std::string, std::string>(property, toString(sub_elem), probability), "$", "^");
            else if(testAttribute(sub_elem, "rdf:datatype"))
            {
              const char* value = sub_elem->GetText();
              if(value != nullptr)
                OntologyReader::push(individual_descriptor.data_relations_, PairElement<std::string, std::string>(property, toString(sub_elem, "rdf:datatype") + "#" + std::string(value), probability), "$", "^");
            }
          }
        }
      }
    }
    graphs_->individuals_.add(node_name, individual_descriptor);
    nb_loaded_elem_++;
  }

  void OntologyOwlReader::readDescription(tinyxml2::XMLElement* elem)
  {
    auto* description_type = elem->FirstChildElement("rdf:type");
    if(description_type == nullptr)
      return;

    const auto* description_type_resource = description_type->Attribute("rdf:resource");
    std::string description_type_resource_name = getName(std::string(description_type_resource));

    if(description_type_resource_name == "AllDisjointClasses")
      readDisjoint(elem, true);
    else if(description_type_resource_name == "AllDisjointProperties")
      readDisjoint(elem, false);
    else if(description_type_resource_name == "Imp")
      readSwrlRule(elem);
  }

  void OntologyOwlReader::readSwrlRule(tinyxml2::XMLElement* elem)
  {
    RuleDescriptor_t rule = readRuleDescription(elem);
    if(!rule.rule_str.empty()) // check if the str expression is not empty, to make sure that the rule exists
    {
      graphs_->rules_.add(rule);
      if(display_)
        std::cout << "│   ├── " + rule.rule_str << std::endl; // todo: mal fait je pense, à mieux intégrer
    }
  }

  void OntologyOwlReader::readDisjoint(tinyxml2::XMLElement* elem, bool is_class)
  {
    std::vector<std::string> disjoints;

    tinyxml2::XMLElement* member_elem = elem->FirstChildElement("owl:members");
    if(member_elem != nullptr)
    {
      const auto* parse_type_member = member_elem->Attribute("rdf:parseType");
      if(parse_type_member != nullptr)
      {
        std::string parse_type = std::string(parse_type_member);
        if(parse_type == "Collection")
          readCollection(disjoints, member_elem, "-");

        if(is_class)
          graphs_->classes_.add(disjoints);
        else
          graphs_->object_properties_.add(disjoints);
      }
    }
  }

  void OntologyOwlReader::readIndividualDescription(tinyxml2::XMLElement* elem)
  {
    std::vector<std::string> distincts;
    bool is_distinct_all = false;
    for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      const std::string sub_elem_name = sub_elem->Value();
      const char* sub_attr = nullptr;
      if(sub_elem_name == "rdf:type")
      {
        sub_attr = sub_elem->Attribute("rdf:resource");
        if(sub_attr != nullptr)
          if(getName(std::string(sub_attr)) == "AllDifferent")
            is_distinct_all = true;
      }
      else if(sub_elem_name == "owl:distinctMembers")
      {
        sub_attr = sub_elem->Attribute("rdf:parseType");
        if(sub_attr != nullptr)
          if(std::string(sub_attr) == "Collection")
            readCollection(distincts, sub_elem, "-");
      }
    }
    if(is_distinct_all)
      graphs_->individuals_.add(distincts);
    distincts.clear();
  }

  void OntologyOwlReader::readObjectProperty(tinyxml2::XMLElement* elem)
  {
    ObjectPropertyDescriptor_t property_vector;
    property_vector.annotation_usage_ = false;
    std::string node_name = getAttribute(elem, std::vector<std::string>{"rdf:about", "rdf:ID"});
    if(node_name.empty() == false)
    {
      if(display_)
        std::cout << "│   ├──" << node_name << std::endl;
      for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      {
        const std::string sub_elem_name = sub_elem->Value();
        const float probability = getProbability(sub_elem);

        if(sub_elem_name == "rdfs:subPropertyOf")
          push(property_vector.mothers_, sub_elem, probability, "+");
        else if(sub_elem_name == "owl:propertyDisjointWith")
          push(property_vector.disjoints_, sub_elem, probability, "-");
        else if(sub_elem_name == "owl:inverseOf")
          push(property_vector.inverses_, sub_elem, probability, "/");
        else if(sub_elem_name == "rdfs:domain")
          push(property_vector.domains_, sub_elem, probability, ">");
        else if(sub_elem_name == "rdfs:range")
          push(property_vector.ranges_, sub_elem, probability, "<");
        else if(sub_elem_name == "rdf:type")
          push(property_vector.properties_, sub_elem, "*");
        else if(sub_elem_name == "rdfs:label")
          pushLang(property_vector.dictionary_, sub_elem);
        else if(sub_elem_name == "onto:label")
          pushLang(property_vector.muted_dictionary_, sub_elem);
        else if(sub_elem_name == "rdfs:comment")
          pushComment(property_vector.comments_, sub_elem);
        else if(sub_elem_name == "owl:propertyChainAxiom")
        {
          std::vector<std::string> tmp;
          readCollection(tmp, sub_elem, ".", 2);
          property_vector.chains_.push_back(tmp);
        }
      }
    }

    graphs_->object_properties_.add(node_name, property_vector);
    nb_loaded_elem_++;
  }

  void OntologyOwlReader::readDataProperty(tinyxml2::XMLElement* elem)
  {
    DataPropertyDescriptor_t property_vector;
    property_vector.annotation_usage_ = false;
    std::string node_name = getAttribute(elem, std::vector<std::string>{"rdf:about", "rdf:ID"});
    if(node_name.empty() == false)
    {
      if(display_)
        std::cout << "│   ├──" << node_name << std::endl;
      for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      {
        const std::string sub_elem_name = sub_elem->Value();
        const float probability = getProbability(sub_elem);

        if(sub_elem_name == "rdfs:subPropertyOf")
          push(property_vector.mothers_, sub_elem, probability, "+");
        else if(sub_elem_name == "owl:propertyDisjointWith")
          push(property_vector.disjoints_, sub_elem, probability, "-");
        else if(sub_elem_name == "rdfs:domain")
          push(property_vector.domains_, sub_elem, probability, ">");
        else if(sub_elem_name == "rdfs:range")
          push(property_vector.ranges_, sub_elem, "<");
        else if(sub_elem_name == "rdfs:label")
          pushLang(property_vector.dictionary_, sub_elem);
        else if(sub_elem_name == "onto:label")
          pushLang(property_vector.muted_dictionary_, sub_elem);
        else if(sub_elem_name == "rdfs:comment")
          pushComment(property_vector.comments_, sub_elem);
      }
    }

    graphs_->data_properties_.add(node_name, property_vector);
    nb_loaded_elem_++;
  }

  void OntologyOwlReader::readAnnotationProperty(tinyxml2::XMLElement* elem)
  {
    DataPropertyDescriptor_t property_vector; // we use a DataPropertyDescriptor_t that is sufficient to represent an annotation property
    property_vector.annotation_usage_ = true;
    std::vector<SingleElement<std::string>> ranges;
    std::string node_name = getAttribute(elem, std::vector<std::string>{"rdf:about", "rdf:ID"});
    if(node_name.empty() == false)
    {
      if(display_)
        std::cout << "│   ├──" << node_name << std::endl;
      for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      {
        const std::string sub_elem_name = sub_elem->Value();
        const float probability = getProbability(sub_elem);

        if(sub_elem_name == "rdfs:subPropertyOf")
          push(property_vector.mothers_, sub_elem, probability, "+");
        else if(sub_elem_name == "owl:disjointWith")
          push(property_vector.disjoints_, sub_elem, probability, "-");
        else if(sub_elem_name == "rdfs:domain")
          push(property_vector.domains_, sub_elem, probability, ">");
        else if(sub_elem_name == "rdfs:range")
        {
          push(property_vector.ranges_, sub_elem, "<");
          push(ranges, sub_elem, probability);
        }
        else if(sub_elem_name == "rdfs:label")
          pushLang(property_vector.dictionary_, sub_elem);
        else if(sub_elem_name == "onto:label")
          pushLang(property_vector.muted_dictionary_, sub_elem);
      }
    }

    // data_property_graph_ will return false if no data property is found with this name
    if(graphs_->data_properties_.addAnnotation(node_name, property_vector) == false)
    {
      ObjectPropertyDescriptor_t object_property_vector;
      object_property_vector.mothers_ = property_vector.mothers_;
      object_property_vector.disjoints_ = property_vector.disjoints_;
      object_property_vector.domains_ = property_vector.domains_;
      object_property_vector.ranges_ = ranges;
      object_property_vector.dictionary_ = property_vector.dictionary_;
      object_property_vector.muted_dictionary_ = property_vector.muted_dictionary_;
      object_property_vector.annotation_usage_ = true;

      graphs_->object_properties_.add(node_name, object_property_vector);
      // if no data property is found, the annotation will be setted as an object property by default
    }

    nb_loaded_elem_++;
  }

  void OntologyOwlReader::readCollection(std::vector<std::string>& vect, tinyxml2::XMLElement* elem, const std::string& symbol, size_t level)
  {
    for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      const std::string sub_elem_name = sub_elem->Value();
      if(sub_elem_name == "rdf:Description")
      {
        const char* sub_attr = sub_elem->Attribute("rdf:about");
        if(sub_attr == nullptr)
          sub_attr = elem->Attribute("rdf:ID");
        if(sub_attr != nullptr)
        {
          if(display_)
          {
            for(size_t i = 0; i < level; i++)
              std::cout << "│   ";
            if(sub_elem == elem->FirstChildElement())
              std::cout << "├───┬── " << symbol;
            else if(sub_elem->NextSiblingElement() == nullptr)
              std::cout << "│   └── " << symbol;
            else
              std::cout << "│   ├── " << symbol;
            std::cout << getName(std::string(sub_attr)) << std::endl;
          }
          vect.push_back(getName(std::string(sub_attr)));
        }
      }
    }
  }

  std::string OntologyOwlReader::readSomeValuesFrom(tinyxml2::XMLElement* elem)
  {
    std::string value = getAttribute(elem, "rdf:resource");
    if(value.empty())
      for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      {
        const std::string restriction_name = sub_elem->Value();
        if(restriction_name == "rdfs:Datatype" && display_)
          std::cout << restriction_name << std::endl;
      }
    return value;
  }

  void OntologyOwlReader::push(Properties_t& properties, tinyxml2::XMLElement* sub_elem, const std::string& symbole, const std::string& attribute)
  {
    const char* sub_attr = sub_elem->Attribute(attribute.c_str());
    if(sub_attr != nullptr)
    {
      std::string property = getName(std::string(sub_attr));
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

      if(property.empty() == false && display_)
        std::cout << "│   │   ├── " << symbole << property << std::endl;
    }
  }

  void OntologyOwlReader::pushLang(std::map<std::string, std::vector<std::string>>& dictionary, tinyxml2::XMLElement* sub_elem)
  {
    const char* sub_attr = sub_elem->Attribute("xml:lang");
    if(sub_attr != nullptr)
    {
      const std::string lang = std::string(sub_attr);

      const char* value = sub_elem->GetText();
      if(value != nullptr)
      {
        dictionary[lang].emplace_back(value);

        if((lang.empty() == false) && (std::string(value).empty() == false) && display_)
          std::cout << "│   │   ├── " << "@" << lang << " : " << dictionary[lang].back() << std::endl;
      }
    }
  }

  void OntologyOwlReader::pushComment(std::map<std::string, std::vector<std::string>>& dictionary, tinyxml2::XMLElement* sub_elem)
  {
    const char* sub_attr = sub_elem->Attribute("xml:lang");
    if(sub_attr != nullptr)
    {
      const std::string lang = std::string(sub_attr);

      const char* value = sub_elem->GetText();
      if(value != nullptr)
      {
        dictionary[lang].emplace_back(value);
        if((lang.empty() == false) && (std::string(value).empty() == false) && display_)
          std::cout << "│   │   ├── " << "#" << lang << " : " << dictionary[lang].back() << std::endl;
      }
    }
  }

  void OntologyOwlReader::removeDocType(std::string& txt)
  {
    size_t pose = txt.find("DOCTYPE");
    if(pose != std::string::npos)
    {
      size_t nb = 1;
      for(size_t i = pose; i < txt.size(); i++)
      {
        if(txt[i] == '<')
          nb++;
        else if(txt[i] == '>')
        {
          nb--;
          if(nb == 0)
          {
            pose = pose - 2;
            txt.erase(pose, i - pose + 1);
            return;
          }
        }
      }
    }
  }

} // namespace ontologenius
