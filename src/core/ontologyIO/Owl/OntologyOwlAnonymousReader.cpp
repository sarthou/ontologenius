#include <string>
#include <tinyxml2.h>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontologyIO/Owl/OntologyOwlReader.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

  ClassExpressionDescriptor_t* OntologyOwlReader::readEquivalentClass(tinyxml2::XMLElement* elem, bool force_datatype, bool is_instanciated, bool take_child)
  {
    auto* class_expression = new ClassExpressionDescriptor_t();
    class_expression->data_usage = force_datatype;
    if(elem->FirstChild() == nullptr) // direct equivalence
    {
      class_expression->resource_value = getAttribute(elem, "rdf:resource");
      if(class_expression->resource_value.empty())
        class_expression->resource_value = getAttribute(elem, std::vector<std::string>{"rdf:about", "rdf:ID"});
      class_expression->type = ClassExpressionType_e::class_expression_identifier;
      class_expression->is_instanciated = is_instanciated;
    }
    else if(take_child)
      readEquivalentClass(elem->FirstChildElement(), class_expression);
    else
      readEquivalentClass(elem, class_expression);

    return class_expression;
  }

  void OntologyOwlReader::readEquivalentClass(tinyxml2::XMLElement* elem, ClassExpressionDescriptor_t* class_expression)
  {
    const std::string elem_name = elem->Value();
    if(elem_name == "owl:Restriction")
      readAnonymousRestriction(elem, class_expression);
    else if(elem_name == "owl:Class")
      readAnonymousClassExpression(elem, class_expression);
    else if(elem_name == "rdfs:Datatype")
    {
      class_expression->data_usage = true;
      readAnonymousClassExpression(elem, class_expression);
    }
  }

  void OntologyOwlReader::readAnonymousRestriction(tinyxml2::XMLElement* elem, ClassExpressionDescriptor_t* class_expression)
  {
    class_expression->type = ClassExpressionType_e::class_expression_restriction;
    for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      bool extract_range = false;
      const std::string sub_elem_name = sub_elem->Value();
      if(sub_elem_name == "owl:onProperty")
      {
        class_expression->restriction_property = getAttribute(sub_elem, std::vector<std::string>{"rdf:resource", "rdf:ID"});
        if(class_expression->restriction_property.empty())
        {
          tinyxml2::XMLElement* property_elem = sub_elem->FirstChildElement();
          if(property_elem != nullptr)
          {
            const std::string property_elem_name = property_elem->Value();
            if(property_elem_name == "owl:ObjectProperty")
            {
              class_expression->restriction_property = getAttribute(property_elem, std::vector<std::string>{"rdf:resource", "rdf:ID"});
            }
            else
              Display::warning("Unsupported field " + property_elem_name + " in owl:onProperty");
          }
        }
      }
      else if(sub_elem_name == "owl:allValuesFrom")
        class_expression->restriction_type = RestrictionConstraintType_e::restriction_all_values_from;
      else if(sub_elem_name == "owl:someValuesFrom")
        class_expression->restriction_type = RestrictionConstraintType_e::restriction_some_values_from;
      else if(sub_elem_name == "owl:hasValue")
        class_expression->restriction_type = RestrictionConstraintType_e::restriction_has_value;
      else if((sub_elem_name == "owl:maxQualifiedCardinality") || (sub_elem_name == "owl:maxCardinality"))
        class_expression->restriction_type = RestrictionConstraintType_e::restriction_max_cardinality;
      else if((sub_elem_name == "owl:minQualifiedCardinality") || (sub_elem_name == "owl:minCardinality"))
        class_expression->restriction_type = RestrictionConstraintType_e::restriction_min_cardinality;
      else if((sub_elem_name == "owl:qualifiedCardinality") || (sub_elem_name == "owl:cardinality"))
        class_expression->restriction_type = RestrictionConstraintType_e::restriction_cardinality;
      else if(sub_elem_name == "owl:onClass")
        extract_range = true;
      else if(sub_elem_name == "owl:onDataRange")
      {
        class_expression->data_usage = true;
        extract_range = true;
      }

      if(extract_range)
      {
        if(getResourceValue(sub_elem, class_expression, false) == false)
          class_expression->sub_expressions.push_back(readEquivalentClass(sub_elem));
      }
      else if((class_expression->restriction_type == RestrictionConstraintType_e::restriction_all_values_from) ||
              (class_expression->restriction_type == RestrictionConstraintType_e::restriction_some_values_from) ||
              (class_expression->restriction_type == RestrictionConstraintType_e::restriction_has_value))
      {
        bool instanciated = (class_expression->restriction_type == RestrictionConstraintType_e::restriction_has_value);
        if(getResourceValue(sub_elem, class_expression, instanciated) == false)
          class_expression->sub_expressions.push_back(readEquivalentClass(sub_elem));
      }
      else if((class_expression->restriction_type == RestrictionConstraintType_e::restriction_max_cardinality) ||
              (class_expression->restriction_type == RestrictionConstraintType_e::restriction_min_cardinality) ||
              (class_expression->restriction_type == RestrictionConstraintType_e::restriction_cardinality))
      {
        std::string type = getAttribute(sub_elem, "rdf:datatype");
        const auto* datavalue = sub_elem->GetText();
        if(datavalue != nullptr)
          class_expression->cardinality_value = type + "#" + std::string(datavalue);
      }
    }
  }

  void OntologyOwlReader::readAnonymousClassExpression(tinyxml2::XMLElement* elem, ClassExpressionDescriptor_t* class_expression)
  {
    auto* expression_type_elem = elem->FirstChildElement();
    if(expression_type_elem != nullptr)
    {
      const std::string expression_type = expression_type_elem->Value();
      if(expression_type == "owl:oneOf")
      {
        class_expression->type = ClassExpressionType_e::class_expression_one_of;
        if(testAttribute(expression_type_elem, "rdf:parseType") == false)
        {
          readDatatypeEnumeration(expression_type_elem, class_expression);
          return;
        }
      }
      else if(expression_type == "owl:complementOf")
        class_expression->type = ClassExpressionType_e::class_expression_complement_of;
      else if(expression_type == "owl:datatypeComplementOf")
      {
        class_expression->type = ClassExpressionType_e::class_expression_complement_of;
        class_expression->data_usage = true;
      }
      else if(expression_type == "owl:intersectionOf")
        class_expression->type = ClassExpressionType_e::class_expression_intersection_of;
      else if(expression_type == "owl:unionOf")
        class_expression->type = ClassExpressionType_e::class_expression_union_of;

      if(class_expression->type == ClassExpressionType_e::class_expression_complement_of)
      {
        std::string single_value = getAttribute(expression_type_elem, "rdf:resource");
        if(single_value.empty() == false)
        {
          class_expression->sub_expressions.emplace_back(new ClassExpressionDescriptor_t());
          class_expression->sub_expressions.back()->data_usage = class_expression->data_usage;
          class_expression->sub_expressions.back()->resource_value = single_value;
          class_expression->sub_expressions.back()->type = ClassExpressionType_e::class_expression_identifier;
          return;
        }
      }

      if(class_expression->type != ClassExpressionType_e::class_expression_unknown)
      {
        bool is_instanciated = (class_expression->type == ClassExpressionType_e::class_expression_one_of);
        for(tinyxml2::XMLElement* sub_elem = expression_type_elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
          class_expression->sub_expressions.push_back(readEquivalentClass(sub_elem, class_expression->data_usage, is_instanciated, false));
      }
    }
  }

  void OntologyOwlReader::readDatatypeEnumeration(tinyxml2::XMLElement* elem, ClassExpressionDescriptor_t* class_expression)
  {
    tinyxml2::XMLElement* elem_to_analyse = nullptr;

    auto* list_elem = elem->FirstChildElement("rdf:List");
    if(list_elem != nullptr)
      elem_to_analyse = list_elem;
    else
    {
      auto* description_elem = elem->FirstChildElement("rdf:Description");
      if(description_elem != nullptr)
      {
        auto* type_elem = description_elem->FirstChildElement("rdf:type");
        if(type_elem != nullptr)
        {
          // should we verify that the type is a List ?
          elem_to_analyse = description_elem;
        }
      }
    }

    if(elem_to_analyse != nullptr)
    {
      auto* first = elem_to_analyse->FirstChildElement("rdf:first");
      std::string type = getAttribute(first, "rdf:datatype");
      if(type.empty())
        type = "string";
      const auto* datavalue = first->GetText();
      if(datavalue != nullptr)
      {
        class_expression->sub_expressions.emplace_back(new ClassExpressionDescriptor_t());
        class_expression->sub_expressions.back()->data_usage = true;
        class_expression->sub_expressions.back()->is_instanciated = true;
        class_expression->sub_expressions.back()->resource_value = type + "#" + std::string(datavalue);
        class_expression->sub_expressions.back()->type = ClassExpressionType_e::class_expression_identifier;
      }

      auto* rest = elem_to_analyse->FirstChildElement("rdf:rest");
      if(testAttribute(rest, "rdf:resource") == false)
        readDatatypeEnumeration(rest, class_expression);
    }
  }

  bool OntologyOwlReader::getResourceValue(tinyxml2::XMLElement* elem, ClassExpressionDescriptor_t* class_expression, bool is_instanciated)
  {
    std::string data_type = getAttribute(elem, "rdf:datatype");
    if(data_type.empty())
    {
      class_expression->resource_value = getAttribute(elem, "rdf:resource");
      if(class_expression->resource_value.empty())
      {
        class_expression->resource_value = getAttribute(elem, "rdf:datatype");
        if(class_expression->resource_value.empty() == false)
          class_expression->data_usage = true;
        else
        {
          const auto* datavalue = elem->GetText();
          if(datavalue != nullptr)
          {
            data_type = "string";
            class_expression->resource_value = data_type + "#" + std::string(datavalue);
            class_expression->data_usage = true;
            class_expression->is_instanciated = true;
            return true;
          }
          else
            return false;
        }
      }
      class_expression->is_instanciated = is_instanciated;
      return true;
    }
    else
    {
      const auto* datavalue = elem->GetText();
      if(datavalue != nullptr)
      {
        class_expression->resource_value = data_type + "#" + std::string(datavalue);
        class_expression->data_usage = true;
        class_expression->is_instanciated = true;
        return true;
      }
      else
        return false;
    }
  }

} // namespace ontologenius