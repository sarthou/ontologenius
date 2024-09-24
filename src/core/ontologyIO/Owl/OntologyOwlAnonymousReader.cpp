#include <string>
#include <tinyxml.h>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontologyIO/Owl/OntologyOwlReader.h"
#include "ontologenius/utils/String.h"

namespace ontologenius {

  void OntologyOwlReader::readEquivalentClass(AnonymousClassVectors_t& ano, TiXmlElement* elem, const std::string& class_name)
  {
    ano.class_equiv = class_name;
    ExpressionMember_t* exp = nullptr;

    // Class only equivalence  : Camera Eq to Component
    if(elem->FirstChild() == nullptr)
    {
      exp = new ExpressionMember_t();
      exp->rest.restriction_range = getName(elem->Attribute("rdf:resource"));
    }
    // Expression equivalence :
    else
    {
      TiXmlElement* sub_elem = elem->FirstChildElement(); // should be the only child element
      const std::string sub_elem_name = sub_elem->Value();

      // Restriction equivalence : Camera Eq to hasComponent some Component
      if(sub_elem_name == "owl:Restriction")
        exp = readAnonymousRestriction(sub_elem);
      // Logical expression equivalence : Camera Eq to (hasComponent some Component and has_node only rational)
      else if(sub_elem_name == "owl:Class")
        exp = readAnonymousClassExpression(sub_elem);
    }

    ano.equivalence_trees.push_back(exp);
    ano.str_equivalences.push_back(exp->toString());
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousRestriction(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();

    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      const std::string sub_elem_name = sub_elem->Value();
      if(sub_elem_name == "owl:onProperty")
        exp->rest.property = getName(sub_elem->Attribute("rdf:resource"));
      else if((sub_elem_name == "owl:maxQualifiedCardinality") ||
              (sub_elem_name == "owl:minQualifiedCardinality") ||
              (sub_elem_name == "owl:qualifiedCardinality"))
        readAnonymousCardinalityValue(sub_elem, exp);
      else if((sub_elem_name == "owl:allValuesFrom") ||
              (sub_elem_name == "owl:hasValue") ||
              (sub_elem_name == "owl:someValuesFrom"))
      {
        if(readAnonymousCardinalityRange(sub_elem, exp) == false)
        {
          exp->is_complex = true;
          addAnonymousChildMember(exp, readAnonymousComplexDescription(sub_elem->FirstChildElement()), sub_elem->FirstChildElement());
        }
      }
      else if(sub_elem_name == "owl:onClass" || sub_elem_name == "owl:onDataRange")
      {
        const char* resource = sub_elem->Attribute("rdf:resource");
        // Simple restriction range : Camera Eq to  hasComponent some Lidar
        // <owl:onClass rdf:resource="test_bastien#Lidar"/>
        if(resource != nullptr)
        {
          const std::string attr_class = sub_elem->Attribute("rdf:resource");
          if(isIn("http://www.w3.org/", attr_class))
          {
            exp->rest.restriction_range = attr_class;
            exp->is_data_property = true;
          }
          else
            exp->rest.restriction_range = getName(attr_class);
        }
        // Complex restriction range with max, min, exactly : Camera Eq to  hasComponent max 2 (not DirtyCutlery)
        // <owl:onClass>
        //     <owl:Class>
        //         <owl:complementOf rdf:resource="test_bastien#DirtyCutlery"/>
        //     </owl:Class>
        // </owl:onClass>
        else
        {
          exp->is_complex = true;
          addAnonymousChildMember(exp, readAnonymousComplexDescription(sub_elem->FirstChildElement()), sub_elem->FirstChildElement());
        }
      }
    }

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousClassExpression(TiXmlElement* elem)
  {
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      const std::string sub_elem_name = sub_elem->Value();
      if(sub_elem_name == "owl:unionOf")
        return readAnonymousUnion(sub_elem);
      else if(sub_elem_name == "owl:intersectionOf")
        return readAnonymousIntersection(sub_elem);
      else if(sub_elem_name == "owl:complementOf")
        return readAnonymousComplement(sub_elem);
      else if(sub_elem_name == "owl:oneOf")
        return readAnonymousOneOf(sub_elem);
    }

    return nullptr;
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousDatatypeExpression(TiXmlElement* elem)
  {
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      ExpressionMember_t* exp = nullptr;
      const std::string sub_elem_name = sub_elem->Value();
      if(sub_elem_name == "owl:unionOf")
        exp = readAnonymousUnion(sub_elem);
      else if(sub_elem_name == "owl:intersectionOf")
        exp = readAnonymousIntersection(sub_elem);
      else if(sub_elem_name == "owl:datatypeComplementOf")
        exp = readAnonymousComplement(sub_elem);

      if(exp != nullptr)
      {
        exp->is_data_property = true;
        return exp;
      }
    }

    return nullptr;
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousIntersection(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->logical_type_ = logical_and;

    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      ExpressionMember_t* child_exp = readAnonymousComplexDescription(sub_elem);
      addAnonymousChildMember(exp, child_exp, sub_elem);
    }

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousUnion(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->logical_type_ = logical_or;

    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      ExpressionMember_t* child_exp = readAnonymousComplexDescription(sub_elem);
      addAnonymousChildMember(exp, child_exp, sub_elem);
    }

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousOneOf(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->oneof = true;

    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      ExpressionMember_t* child_exp = readAnonymousResource(sub_elem, "rdf:about");
      addAnonymousChildMember(exp, child_exp, sub_elem);
    }

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousComplement(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->logical_type_ = logical_not;
    ExpressionMember_t* child_exp = nullptr;

    const char* resource = elem->Attribute("rdf:resource");

    if(resource != nullptr)
      child_exp = readAnonymousResource(elem);
    else
      child_exp = readAnonymousComplexDescription(elem->FirstChildElement());

    addAnonymousChildMember(exp, child_exp, elem->FirstChildElement());

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousComplexDescription(TiXmlElement* elem)
  {
    if(elem == nullptr)
      return nullptr;

    const std::string elem_name = elem->Value();
    if(elem_name == "owl:Class")
      return readAnonymousClassExpression(elem);
    else if(elem_name == "rdfs:Datatype")
      return readAnonymousDatatypeExpression(elem);
    else if(elem_name == "owl:Restriction")
      return readAnonymousRestriction(elem);
    else if(elem_name == "rdf:Description")
      return readAnonymousResource(elem, "rdf:about");
    else
      return nullptr;
  }

  ExpressionMember_t* OntologyOwlReader::readAnonymousResource(TiXmlElement* elem, const std::string& attribute_name)
  {
    ExpressionMember_t* exp = nullptr;

    const char* resource = elem->Attribute(attribute_name.c_str());
    if(resource != nullptr)
    {
      exp = new ExpressionMember_t();

      const std::string attr_class = elem->Attribute(attribute_name.c_str());
      if(isIn("http://www.w3.org/", attr_class))
      {
        exp->rest.restriction_range = attr_class;
        exp->is_data_property = true;
      }
      else
        exp->rest.restriction_range = getName(attr_class);
    }

    return exp;
  }

  void OntologyOwlReader::addAnonymousChildMember(ExpressionMember_t* parent, ExpressionMember_t* child, TiXmlElement* used_elem)
  {
    if(child != nullptr)
    {
      parent->child_members.push_back(child);
      child->mother = parent;
      if((used_elem != nullptr) && (std::string(used_elem->Value()) != "owl:Restriction"))
        parent->is_data_property = child->is_data_property;
    }
  }

  void OntologyOwlReader::readAnonymousCardinalityValue(TiXmlElement* elem, ExpressionMember_t* exp)
  {
    const std::string sub_elem_name = elem->Value();

    exp->rest.card.cardinality_type = card_map_[sub_elem_name];

    if(elem->GetText() != nullptr)
      exp->rest.card.cardinality_number = elem->GetText();
  }

  bool OntologyOwlReader::readAnonymousCardinalityRange(TiXmlElement* elem, ExpressionMember_t* exp)
  {
    const std::string sub_elem_name = elem->Value();

    const char* resource = elem->Attribute("rdf:resource");
    exp->rest.card.cardinality_type = card_map_[sub_elem_name];

    if(resource != nullptr)
    {
      const std::string s = resource;
      if(isIn("http://www.w3.org/", s))
      {
        exp->rest.card.cardinality_range = resource;
        exp->is_data_property = true;
      }
      else
        exp->rest.card.cardinality_range = getName(resource);
      return true;
    }
    else
      return false;
  }

} // namespace ontologenius