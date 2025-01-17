#include <cstddef>
#include <iostream>
#include <string>
#include <tinyxml.h>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"
#include "ontologenius/core/ontologyIO/Owl/OntologyOwlReader.h"
#include "ontologenius/utils/String.h"

namespace ontologenius {

  void OntologyOwlReader::readRuleDescription(Rule_t& rule, TiXmlElement* elem)
  {
    std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>> exp_body, exp_head;
    // read body
    auto* rule_body = elem->FirstChildElement("swrl:body");
    if(rule_body == nullptr)
      return;
    readRuleCollection(rule_body, exp_body);
    rule.antecedents = exp_body;

    // read head
    auto* rule_head = elem->FirstChildElement("swrl:head");
    if(rule_head == nullptr)
      return;
    readRuleCollection(rule_head, exp_head);
    rule.consequents = exp_head;

    // compute the full str rule
    rule.rule_str = rule.toStringRule();
  }

  void OntologyOwlReader::readRuleCollection(TiXmlElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>>& exp_vect)
  {
    auto* sub_elem = elem->FirstChildElement("rdf:Description");
    if(sub_elem != nullptr)
      readRuleCollection(sub_elem, exp_vect);
    else
    {
      // Atom Type
      std::pair<ontologenius::ExpressionMember_t*, std::vector<Variable_t>> res;
      auto* type_elem = elem->FirstChildElement("rdf:type");
      if(type_elem != nullptr)
      {
        const char* type_resource = nullptr;
        type_resource = type_elem->Attribute("rdf:resource");
        if(type_resource != nullptr)
        {
          std::string type_name = getName(std::string(type_resource));
          if(type_name == "AtomList")
          {
            // read first elem
            auto* first_elem = elem->FirstChildElement("rdf:first");
            if(first_elem != nullptr)
              readFirstAtom(first_elem, exp_vect);
            else
              return;
            // read rest elem
            auto* rest_elem = elem->FirstChildElement("rdf:rest");
            if(rest_elem != nullptr)
              readRestAtom(rest_elem, exp_vect);
            else
              return;
          }
          else
          {
            res = readRuleAtom(elem, type_name);
            exp_vect.push_back(res);
          }
        }
      }
      else
        return;
    }
  }

  std::pair<ExpressionMember_t*, std::vector<Variable_t>> OntologyOwlReader::readRuleAtom(TiXmlElement* elem, const std::string& type_atom)
  {
    std::pair<ExpressionMember_t*, std::vector<Variable_t>> res;

    if(type_atom == "ClassAtom")
      res = readRuleClassAtom(elem);
    else if(type_atom == "IndividualPropertyAtom")
      res = readRuleObjectPropertyAtom(elem);
    else if(type_atom == "DatavaluedPropertyAtom")
      res = readRuleDataPropertyAtom(elem);
    else if(type_atom == "BuiltinAtom")
      res = readRuleBuiltinAtom(elem);

    return res;
  }

  void OntologyOwlReader::readFirstAtom(TiXmlElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>>& exp_vect)
  {
    readRuleCollection(elem, exp_vect);
  }

  void OntologyOwlReader::readRestAtom(TiXmlElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>>& exp_vect)
  {
    const char* type_rest = nullptr;
    type_rest = elem->Attribute("rdf:resource");
    if(type_rest != nullptr)
    {
      std::string rest_name = getName(std::string(type_rest));
      if(rest_name == "nil")
        return;
    }
    else
      readRuleCollection(elem, exp_vect);
  }

  std::pair<ExpressionMember_t*, std::vector<Variable_t>> OntologyOwlReader::readRuleClassAtom(TiXmlElement* elem)
  {
    std::vector<Variable_t> variables;
    ExpressionMember_t* temp_exp = nullptr;

    auto* class_pred = elem->FirstChildElement("swrl:classPredicate");

    if(class_pred != nullptr)
    {
      //  check for the class expression
      const char* class_resource = nullptr;
      class_resource = class_pred->Attribute("rdf:resource");

      if(class_resource != nullptr)
        temp_exp = readRuleResource(class_pred, "rdf:resource");
      else
      {
        //  complex class expression
        auto* restriction_elem = class_pred->FirstChildElement("owl:Restriction");   // simple restriction : hasComponent some Component
        auto* complex_restriction_elem = class_pred->FirstChildElement("owl:Class"); // complex restriction : (hasComponent some Component and has_node only rational)

        if(restriction_elem != nullptr)
          temp_exp = readRuleRestriction(restriction_elem);
        else if(complex_restriction_elem != nullptr)
          temp_exp = readRuleClassExpression(complex_restriction_elem);
      }

      // get argument 1
      auto* var_arg = elem->FirstChildElement("swrl:argument1");
      if(var_arg != nullptr)
        variables.push_back(getRuleArgument(var_arg));
    }
    return {std::make_pair(temp_exp, variables)};
  }

  std::pair<ExpressionMember_t*, std::vector<Variable_t>> OntologyOwlReader::readRuleObjectPropertyAtom(TiXmlElement* elem)
  {
    std::vector<Variable_t> variables;
    ExpressionMember_t* temp_exp = new ExpressionMember_t();

    // std::cout << "ObjectPropertyAtom" << std::endl;
    auto* obj_pred = elem->FirstChildElement("swrl:propertyPredicate");

    // get Object Property
    if(obj_pred != nullptr)
    {
      const auto* obj_prop = obj_pred->Attribute("rdf:resource");
      if(obj_prop != nullptr)
        temp_exp->rest.property = getName(std::string(obj_prop));
    }

    // get argument 1
    auto* var_arg1 = elem->FirstChildElement("swrl:argument1");
    if(var_arg1 != nullptr)
      variables.push_back(getRuleArgument(var_arg1));

    // get argument 2
    auto* var_arg2 = elem->FirstChildElement("swrl:argument2");
    if(var_arg2 != nullptr)
      variables.push_back(getRuleArgument(var_arg2));

    return {std::make_pair(temp_exp, variables)};
  }

  std::pair<ExpressionMember_t*, std::vector<Variable_t>> OntologyOwlReader::readRuleDataPropertyAtom(TiXmlElement* elem)
  {
    std::vector<Variable_t> variables;
    ExpressionMember_t* temp_exp = new ExpressionMember_t();

    // std::cout << "DataPropertyAtom" << std::endl;auto* sub_elem = elem->FirstChildElement("rdf:Description");
    auto* data_pred = elem->FirstChildElement("swrl:propertyPredicate");

    // get Data Property
    if(data_pred != nullptr)
    {
      const auto* data_prop = data_pred->Attribute("rdf:resource");
      if(data_prop != nullptr)
      {
        temp_exp->rest.property = getName(std::string(data_prop));
        temp_exp->is_data_property = true;
      }
    }

    // get argument 1
    auto* var_arg1 = elem->FirstChildElement("swrl:argument1");
    if(var_arg1 != nullptr)
      variables.push_back(getRuleArgument(var_arg1));

    // get argument 2
    auto* var_arg2 = elem->FirstChildElement("swrl:argument2");
    if(var_arg2 != nullptr)
      variables.push_back(getRuleArgument(var_arg2));

    return {std::make_pair(temp_exp, variables)};
  }

  Variable_t OntologyOwlReader::getRuleArgument(TiXmlElement* elem)
  {
    Variable_t new_var;
    const auto* var_name_resource = elem->Attribute("rdf:resource");
    const auto* var_name_dataype = elem->Attribute("rdf:datatype");

    if(var_name_resource != nullptr)
    {
      if(isIn("swrl:var", var_name_resource))
      {
        new_var.var_name = getName(std::string(var_name_resource));
        new_var.is_instantiated = false;
      }
      else
      {
        new_var.var_name = getName(std::string(var_name_resource));
        new_var.is_instantiated = true;
      }
    }
    else if(var_name_dataype != nullptr)
    {
      new_var.var_name = getName(std::string(var_name_dataype) + "#" + std::string(elem->GetText()));
      new_var.is_datavalue = true;
    }

    return new_var;
  }

  std::pair<ExpressionMember_t*, std::vector<Variable_t>> OntologyOwlReader::readRuleBuiltinAtom(TiXmlElement* elem)
  {
    // std::cout << "BuiltinAtom" << std::endl;
    std::vector<Variable_t> variables;
    ExpressionMember_t* temp_exp = new ExpressionMember_t();

    // get builtin type
    const auto* builtin_type = elem->FirstChildElement("swrl:builtin")->Attribute("rdf:resource");

    if(builtin_type != nullptr)
    {
      std::string builtin_name = getName(std::string(builtin_type));
      std::cout << builtin_name << std::endl;
      if(builtin_name == "greaterThan")
        temp_exp->builtin_type_ = greaterThan;
      else if(builtin_name == "greaterThanOrEqual")
        temp_exp->builtin_type_ = greaterThanOrEqual;
      else if(builtin_name == "lessThan")
        temp_exp->builtin_type_ = lessThan;
      else if(builtin_name == "lessThanOrEqual")
        temp_exp->builtin_type_ = lessThanOrEqual;
      else if(builtin_name == "equal")
        temp_exp->builtin_type_ = equal;
      else if(builtin_name == "notEqual")
        temp_exp->builtin_type_ = notEqual;
      else
      {
        temp_exp->builtin_type_ = builtin_none;
        if(display_)
          std::cout << "unsupported buitlin atom : " << builtin_name << std::endl;
      }
    }
    // get builtins arguments in order
    auto* builtin_arguments = elem->FirstChildElement("swrl:arguments");

    variables = readRuleBuiltinArguments(builtin_arguments);

    return {std::make_pair(temp_exp, variables)};
  }

  std::vector<Variable_t> OntologyOwlReader::readRuleBuiltinArguments(TiXmlElement* elem)
  {
    std::vector<Variable_t> variables;

    if(elem->Attribute("rdf:parseType") != nullptr) // two variables
      readSimpleBuiltinArguments(elem, variables);
    else
      readComplexBuiltinArguments(elem, variables);

    return variables;
  }

  void OntologyOwlReader::readSimpleBuiltinArguments(TiXmlElement* elem, std::vector<Variable_t>& variables)
  {
    for(TiXmlElement* sub_elem = elem->FirstChildElement("rdf:Description");
        sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement("rdf:Description"))
    {
      const auto* builtin_arg = elem->Attribute("rdf:about");
      if(builtin_arg != nullptr)
      {
        Variable_t new_var;
        new_var.var_name = getName(std::string(builtin_arg));
        new_var.is_instantiated = false;
        new_var.is_datavalue = false;
        new_var.is_builtin_value = true;
        variables.push_back(new_var);
      }
    }
  }

  void OntologyOwlReader::readComplexBuiltinArguments(TiXmlElement* elem, std::vector<Variable_t>& variables)
  {
    auto* sub_elem = elem->FirstChildElement("rdf:Description");

    if(sub_elem != nullptr)
      readComplexBuiltinArguments(sub_elem, variables);
    else
    {
      const auto* new_elem = elem->FirstChildElement("rdf:first");
      if(new_elem != nullptr)
      {
        const auto* var_elem = new_elem->Attribute("rdf:resource");
        const auto* value_elem = new_elem->Attribute("rdf:datatype");

        if(var_elem != nullptr)
        {
          Variable_t new_var;
          new_var.var_name = getName(std::string(var_elem));
          variables.push_back(new_var);
        }
        else if(value_elem != nullptr)
        {
          Variable_t new_var;
          new_var.var_name = getName(std::string(value_elem) + "#" + std::string(new_elem->GetText()));
          new_var.is_datavalue = true;
          variables.push_back(new_var);
        }

        auto* rest_elem = elem->FirstChildElement("rdf:rest");
        if(rest_elem->Attribute("rdf:resource") == nullptr)
          readComplexBuiltinArguments(rest_elem, variables);
      }
      else
        readComplexBuiltinArguments(elem, variables);
    }
  }

  ExpressionMember_t* OntologyOwlReader::readRuleRestriction(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();

    // get property
    auto* property_elem = elem->FirstChildElement("owl:onProperty");
    if(property_elem != nullptr)
      exp->rest.property = getName(property_elem->Attribute("rdf:resource"));

    // get cardinality
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      const std::string sub_elem_name = sub_elem->Value();
      if((sub_elem_name == "owl:maxQualifiedCardinality") ||
         (sub_elem_name == "owl:minQualifiedCardinality") ||
         (sub_elem_name == "owl:qualifiedCardinality"))
      {
        readRuleCardinalityValue(sub_elem, exp);
        break;
      }
      else if((sub_elem_name == "owl:allValuesFrom") ||
              (sub_elem_name == "owl:hasValue") ||
              (sub_elem_name == "owl:someValuesFrom"))
      {
        if(readRuleCardinalityRange(sub_elem, exp) == false)
        {
          exp->is_complex = true;
          addRuleChildMember(exp, readRuleComplexDescription(sub_elem->FirstChildElement()), sub_elem->FirstChildElement());
          break;
        }
      }
    }

    // get range
    auto* class_elem = elem->FirstChildElement("owl:onClass");
    auto* data_elem = elem->FirstChildElement("owl:onDataRange");

    if(class_elem != nullptr)
    {
      exp->is_data_property = false;
      const char* resource = class_elem->Attribute("rdf:resource");
      // Simple restriction range : Camera Eq to  hasComponent some Lidar
      if(resource != nullptr)
        exp->rest.restriction_range = getName(resource);
      // Complex restriction range with max, min, exactly : Camera Eq to  hasComponent max 2 (not DirtyCutlery)
      else
      {
        exp->is_complex = true;
        addRuleChildMember(exp, readRuleComplexDescription(class_elem->FirstChildElement()), class_elem->FirstChildElement());
      }
    }
    else if(data_elem != nullptr)
    {
      exp->is_data_property = true;
      const char* resource = data_elem->Attribute("rdf:resource");
      // Simple restriction range : Camera Eq to  has_node some boolean
      if(resource != nullptr)
        exp->rest.restriction_range = resource;
      // Complex restriction range with max, min, exactly : Camera Eq to  has_node some (not(boolean))
      else
      {
        exp->is_complex = true;
        addRuleChildMember(exp, readRuleComplexDescription(data_elem->FirstChildElement()), data_elem->FirstChildElement());
      }
    }
    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readRuleClassExpression(TiXmlElement* elem)
  {
    // check for type of node
    auto* union_node = elem->FirstChildElement("owl:unionOf");
    if(union_node != nullptr)
      return readRuleUnion(union_node);

    auto* inter_node = elem->FirstChildElement("owl:intersectionOf");
    if(inter_node != nullptr)
      return readRuleIntersection(inter_node);

    auto* negat_node = elem->FirstChildElement("owl:complementOf");
    if(negat_node != nullptr)
      return readRuleComplement(negat_node);

    auto* oneof_node = elem->FirstChildElement("owl:oneOf");
    if(oneof_node != nullptr)
      return readRuleOneOf(oneof_node);

    return nullptr;
  }

  ExpressionMember_t* OntologyOwlReader::readRuleDatatypeExpression(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = nullptr;
    // check for type of node
    auto* union_node = elem->FirstChildElement("owl:unionOf");
    if(union_node != nullptr)
      exp = readRuleUnion(union_node);

    auto* inter_node = elem->FirstChildElement("owl:intersectionOf");
    if(inter_node != nullptr)
      exp = readRuleIntersection(inter_node);

    auto* negat_node = elem->FirstChildElement("owl:datatypeComplementOf");
    if(negat_node != nullptr)
      exp = readRuleComplement(negat_node);

    if(exp != nullptr)
    {
      exp->is_data_property = true;
      return exp;
    }
    return nullptr;
  }

  ExpressionMember_t* OntologyOwlReader::readRuleIntersection(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->logical_type_ = logical_and;

    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      ExpressionMember_t* child_exp = readRuleComplexDescription(sub_elem);
      addRuleChildMember(exp, child_exp, sub_elem);
    }

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readRuleUnion(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->logical_type_ = logical_or;

    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      ExpressionMember_t* child_exp = readRuleComplexDescription(sub_elem);
      addRuleChildMember(exp, child_exp, sub_elem);
    }

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readRuleOneOf(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->oneof = true;

    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      ExpressionMember_t* child_exp = readRuleResource(sub_elem, "rdf:about");
      addRuleChildMember(exp, child_exp, sub_elem);
    }

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readRuleComplement(TiXmlElement* elem)
  {
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->logical_type_ = logical_not;
    ExpressionMember_t* child_exp = nullptr;

    const char* resource = elem->Attribute("rdf:resource");

    if(resource != nullptr)
      child_exp = readRuleResource(elem);
    else
      child_exp = readRuleComplexDescription(elem->FirstChildElement());

    addRuleChildMember(exp, child_exp, elem->FirstChildElement());

    return exp;
  }

  ExpressionMember_t* OntologyOwlReader::readRuleComplexDescription(TiXmlElement* elem)
  {
    if(elem == nullptr)
      return nullptr;

    const std::string elem_name = elem->Value();
    if(elem_name == "owl:Class")
      return readRuleClassExpression(elem);
    else if(elem_name == "rdfs:Datatype")
      return readRuleDatatypeExpression(elem);
    else if(elem_name == "owl:Restriction")
      return readRuleRestriction(elem);
    else if(elem_name == "rdf:Description")
      return readRuleResource(elem, "rdf:about");
    else
      return nullptr;
  }

  ExpressionMember_t* OntologyOwlReader::readRuleResource(TiXmlElement* elem, const std::string& attribute_name)
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

  void OntologyOwlReader::addRuleChildMember(ExpressionMember_t* parent, ExpressionMember_t* child, TiXmlElement* used_elem)
  {
    if(child != nullptr)
    {
      parent->child_members.push_back(child);
      child->mother = parent;
      if((used_elem != nullptr) && (std::string(used_elem->Value()) != "owl:Restriction"))
        parent->is_data_property = child->is_data_property;
    }
  }

  void OntologyOwlReader::readRuleCardinalityValue(TiXmlElement* elem, ExpressionMember_t* exp)
  {
    const std::string sub_elem_name = elem->Value();

    exp->rest.card.cardinality_type = card_map_[sub_elem_name];

    if(elem->GetText() != nullptr)
      exp->rest.card.cardinality_number = elem->GetText();
  }

  bool OntologyOwlReader::readRuleCardinalityRange(TiXmlElement* elem, ExpressionMember_t* exp)
  {
    const std::string sub_elem_name = elem->Value();

    const char* resource = elem->Attribute("rdf:resource");
    const char* resource_data = elem->Attribute("rdf:datatype");
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
    else if(resource_data != nullptr)
    {
      exp->is_data_property = true;
      exp->rest.card.cardinality_range = getName(resource_data) + "#" + elem->GetText();
      return true;
    }
    return false;
  }

} // namespace ontologenius