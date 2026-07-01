#include <cstddef>
#include <iostream>
#include <string>
#include <tinyxml2.h>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"
#include "ontologenius/core/ontologyIO/Owl/OntologyOwlReader.h"
#include "ontologenius/utils/String.h"

namespace ontologenius {

  RuleDescriptor_t OntologyOwlReader::readRuleDescription(tinyxml2::XMLElement* elem)
  {
    RuleDescriptor_t rule;

    // read comment
    auto* rule_comment = elem->FirstChildElement("rdfs:comment");
    if(rule_comment != nullptr)
      pushComment(rule.comments_, rule_comment);

    // read body
    auto* rule_body = elem->FirstChildElement("swrl:body");
    if(rule_body == nullptr)
      return rule;
    readRuleCollection(rule_body, rule.antecedents);

    // read head
    auto* rule_head = elem->FirstChildElement("swrl:head");
    if(rule_head == nullptr)
      return rule;
    readRuleCollection(rule_head, rule.consequents);

    rule.rule_str = rule.toString();

    return rule;
  }

  void OntologyOwlReader::readRuleCollection(tinyxml2::XMLElement* elem, std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>>& atoms)
  {
    auto* description_elem = elem->FirstChildElement("rdf:Description");
    if(description_elem == nullptr)
      return;

    auto* type_elem = description_elem->FirstChildElement("rdf:type");
    if(type_elem != nullptr)
    {
      std::string description_type = getAttribute(type_elem, "rdf:resource");
      if(description_type == "AtomList")
        readAtomList(description_elem, atoms);
      else if(description_type == "ClassAtom")
        readClassAtom(description_elem, atoms);
      else if(description_type == "IndividualPropertyAtom")
        readIndividualPropertyAtom(description_elem, atoms);
      else if(description_type == "DatavaluedPropertyAtom")
        readDatavaluedPropertyAtom(description_elem, atoms);
      else if(description_type == "BuiltinAtom")
        readBuiltinAtom(description_elem, atoms);
      else
        std::cout << "Unknown description type " << description_type << std::endl;
    }
  }

  void OntologyOwlReader::readAtomList(tinyxml2::XMLElement* elem, std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>>& atoms)
  {
    auto* first_elem = elem->FirstChildElement("rdf:first");
    if(first_elem != nullptr)
    {
      readRuleCollection(first_elem, atoms);
      auto* rest_elem = elem->FirstChildElement("rdf:rest");
      if(rest_elem != nullptr)
      {
        if(rest_elem->Attribute("rdf:resource") == nullptr)
          readRuleCollection(rest_elem, atoms);
      }
    }
  }

  void OntologyOwlReader::readClassAtom(tinyxml2::XMLElement* elem, std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>>& atoms)
  {
    auto* class_pred_elem = elem->FirstChildElement("swrl:classPredicate");
    if(class_pred_elem != nullptr)
    {
      atoms.emplace_back();
      auto& atom = atoms.back();
      atom.first.type = RuleAtomType_e::rule_atom_class;

      std::string resource = getAttribute(class_pred_elem, "rdf:resource");
      if(resource.empty() == false)
        atom.first.resource_value = std::move(resource);
      else // Anonymous class
        atom.first.class_expression = readEquivalentClass(class_pred_elem);

      readRuleVariables(elem, atom.second);
    }
  }

  void OntologyOwlReader::readIndividualPropertyAtom(tinyxml2::XMLElement* elem, std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>>& atoms)
  {
    auto* property_predicate_elem = elem->FirstChildElement("swrl:propertyPredicate");
    if(property_predicate_elem != nullptr)
    {
      atoms.emplace_back();
      auto& atom = atoms.back();
      atom.first.type = RuleAtomType_e::rule_atom_object;
      atom.first.resource_value = getAttribute(property_predicate_elem, "rdf:resource");
      readRuleVariables(elem, atom.second);
    }
  }

  void OntologyOwlReader::readDatavaluedPropertyAtom(tinyxml2::XMLElement* elem, std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>>& atoms)
  {
    auto* property_predicate_elem = elem->FirstChildElement("swrl:propertyPredicate");
    if(property_predicate_elem != nullptr)
    {
      atoms.emplace_back();
      auto& atom = atoms.back();
      atom.first.type = RuleAtomType_e::rule_atom_data;
      atom.first.resource_value = getAttribute(property_predicate_elem, "rdf:resource");
      readRuleVariables(elem, atom.second);
    }
  }

  void OntologyOwlReader::readBuiltinAtom(tinyxml2::XMLElement* elem, std::vector<std::pair<RuleAtomDescriptor_t, std::vector<RuleVariableDescriptor_t>>>& atoms)
  {
    auto* builtin_elem = elem->FirstChildElement("swrl:builtin");
    if(builtin_elem != nullptr)
    {
      atoms.emplace_back();
      auto& atom = atoms.back();
      atom.first.type = RuleAtomType_e::rule_atom_builtin;
      std::string builtin_str = getAttribute(builtin_elem, "rdf:resource");

      if(builtin_str == "greaterThan")
        atom.first.builtin = RuleBuiltinType_e::builtin_greater_than;
      else if(builtin_str == "greaterThanOrEqual")
        atom.first.builtin = RuleBuiltinType_e::builtin_greater_than_or_equal;
      else if(builtin_str == "lessThan")
        atom.first.builtin = RuleBuiltinType_e::builtin_less_than;
      else if(builtin_str == "lessThanOrEqual")
        atom.first.builtin = RuleBuiltinType_e::builtin_less_than_or_equal;
      else if(builtin_str == "equal")
        atom.first.builtin = RuleBuiltinType_e::builtin_equal;
      else if(builtin_str == "notEqual")
        atom.first.builtin = RuleBuiltinType_e::builtin_not_equal;
      else if(display_)
        std::cout << "unsupported buitlin atom : " << builtin_str << std::endl;

      auto* builtin_arguments = elem->FirstChildElement("swrl:arguments");
      readRuleBuiltinArguments(builtin_arguments, atom.second);
    }
  }

  void OntologyOwlReader::readRuleVariables(tinyxml2::XMLElement* elem, std::vector<RuleVariableDescriptor_t>& variables)
  {
    for(size_t index = 1;; index++)
    {
      std::string arg_name = "swrl:argument" + std::to_string(index);
      auto* var_arg = elem->FirstChildElement(arg_name.c_str());
      if(var_arg != nullptr)
        variables.push_back(getRuleArgument(var_arg));
      else
        break;
    }
  }

  RuleVariableDescriptor_t OntologyOwlReader::getRuleArgument(tinyxml2::XMLElement* elem)
  {
    RuleVariableDescriptor_t new_var;
    const auto* var_name_resource = elem->Attribute("rdf:resource");
    const auto* var_name_dataype = elem->Attribute("rdf:datatype");

    if(var_name_resource != nullptr)
    {
      new_var.name = getName(std::string(var_name_resource));
      new_var.is_instanciated = !isIn("swrl:var", var_name_resource);
    }
    else if(var_name_dataype != nullptr)
    {
      new_var.name = getName(std::string(var_name_dataype) + "#" + std::string(elem->GetText()));
      new_var.is_instanciated = true;
      new_var.datatype = true;
    }

    return new_var;
  }

  void OntologyOwlReader::readRuleBuiltinArguments(tinyxml2::XMLElement* elem, std::vector<RuleVariableDescriptor_t>& variables)
  {
    const auto* parsing_type_attribute = elem->Attribute("rdf:parseType");
    if(parsing_type_attribute != nullptr)
    {
      std::string parsing_type = std::string(parsing_type_attribute);
      if(parsing_type == "Collection")
        readBuiltinArgumentsCollection(elem, variables);
      // else // todo: error unsuported parsing type
    }
    else
      readBuiltinArgumentsList(elem, variables);
  }

  void OntologyOwlReader::readBuiltinArgumentsCollection(tinyxml2::XMLElement* elem, std::vector<RuleVariableDescriptor_t>& variables)
  {
    for(tinyxml2::XMLElement* description_elem = elem->FirstChildElement("rdf:Description");
        description_elem != nullptr; description_elem = description_elem->NextSiblingElement("rdf:Description"))
    {
      const auto* builtin_arg = description_elem->Attribute("rdf:about");
      if(builtin_arg != nullptr)
      {
        variables.emplace_back(); // create an empty variable at the end
        variables.back().name = std::string(builtin_arg);
      }
    }
  }

  void OntologyOwlReader::readBuiltinArgumentsList(tinyxml2::XMLElement* elem, std::vector<RuleVariableDescriptor_t>& variables)
  {
    auto* description = elem->FirstChildElement("rdf:Description");
    if(description == nullptr)
      return; // should never happend

    auto* type_element = description->FirstChildElement("rdf:type");
    if(type_element == nullptr)
      return; // should never happend

    std::string type = getAttribute(type_element, "rdf:resource");
    if(type == "List")
    {
      auto* first_element = description->FirstChildElement("rdf:first");
      if(first_element != nullptr)
      {
        std::string attribute = getAttribute(first_element, "rdf:resource");
        if(attribute.empty() == false)
        {
          variables.emplace_back(); // create an empty variable at the end
          variables.back().name = attribute;
          variables.back().is_instanciated = !isIn("swrl:var", first_element->Attribute("rdf:resource"));
        }
        else
        {
          attribute = getAttribute(first_element, "rdf:datatype");
          if(attribute.empty() == false)
          {
            variables.emplace_back(); // create an empty variable at the end
            variables.back().datatype = true;
            variables.back().name = attribute + "#" + std::string(first_element->GetText());
            variables.back().is_instanciated = true;
          }
        }
      }

      auto* rest_element = description->FirstChildElement("rdf:rest");
      if(rest_element != nullptr)
      {
        const auto* resource = rest_element->Attribute("rdf:resource");
        if(resource == nullptr) // if not nullptr should be nil
          readRuleBuiltinArguments(rest_element, variables);
      }
    }
  }

} // namespace ontologenius