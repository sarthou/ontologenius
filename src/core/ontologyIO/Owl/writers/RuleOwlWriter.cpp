#include "ontologenius/core/ontologyIO/Owl/writers/RuleOwlWriter.h"

#include <shared_mutex>

#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

  RuleOwlWriter::RuleOwlWriter(RuleGraph* rule_graph, const std::string& ns) : rule_graph_(rule_graph)
  {
    ns_ = ns;
  }

  void RuleOwlWriter::write(FILE* file)
  {
    file_ = file;

    const std::shared_lock<std::shared_timed_mutex> lock(rule_graph_->mutex_);

    const std::vector<RuleBranch*> rules = rule_graph_->get();

    // write all the variables involved in the rules
    for(auto& var : rule_graph_->variable_names_)
      writeVariable(var);

    // write the rules
    for(auto* rule : rules)
      writeRule(rule);

    file_ = nullptr;
  }

  void RuleOwlWriter::writeRule(RuleBranch* branch)
  {
    const size_t level = 1;
    std::string field;
    field = "rdf:Description";

    writeString("<" + field + ">\n", level);
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#Imp\"/>\n", level + 1);

    // write the body of the rule
    writeString("<swrl:body>\n", level + 1);

    writeAtomList(branch->rule_antecedents_, level + 2);

    writeString("</swrl:body>\n", level + 1);

    // write the head of the rule
    writeString("<swrl:head>\n", level + 1);

    writeAtomList(branch->rule_consequents_, level + 2);

    writeString("</swrl:head>\n", level + 1);

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeVariable(std::string rule_variable)
  {
    const size_t level = 1;
    std::string field, tmp;
    field = "rdf:Description";

    writeString("<" + field + " rdf:about=\"urn:swrl:var#" + rule_variable + "\">\n", level);
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#Variable\"/>\n", level + 1);
    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeAtom(ClassAtom* class_atom, size_t level)
  {
    std::string tmp, field_name;
    field_name = "swrl:classPredicate";

    // writing the Atom Type
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#ClassAtom\"/>\n", level);

    tmp = "<" + field_name;

    // writing the ClassPredicate
    if(class_atom->class_expression->logical_type_ != logical_none) // Collection
    {
      tmp += ">\n";
      writeString(tmp, level);

      writeClassExpression(class_atom->class_expression, level + 1);

      writeString("</" + field_name + ">\n", level);
    }
    else if(class_atom->class_expression->is_complex || class_atom->class_expression->oneof ||
            class_atom->class_expression->data_property_involved_ ||
            class_atom->class_expression->object_property_involved_) // complex class
    {
      tmp += ">\n";
      writeString(tmp, level);

      writeRestriction(class_atom->class_expression, level + 1);

      writeString("</" + field_name + ">\n", level);
    }
    else if(class_atom->class_expression->class_involved_ != nullptr)
    {
      tmp += " rdf:resource=\"" + ns_ + "#" + class_atom->class_expression->class_involved_->value() + "\"/>\n"; // single class expression
      writeString(tmp, level);
    }

    // writing the Variable
    // <swrl:argument1 rdf:resource="urn:swrl:var#x"/>
    tmp = "<swrl:argument1 rdf:resource=";
    if(class_atom->individual_involved == nullptr)
      tmp += "\"urn:swrl:var#" + class_atom->var;
    else
      tmp += "\"" + ns_ + "#" + class_atom->individual_involved->value();

    tmp += "\"/>\n";
    writeString(tmp, level);
  }

  void RuleOwlWriter::writeAtom(ObjectPropertyAtom* object_atom, size_t level)
  {
    std::string field, tmp, subfield_prop, subfield_var;
    field = "rdf:Description";
    subfield_prop = "swrl:propertyPredicate";
    subfield_var = "swrl:argument";

    // writing the Atom type
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#IndividualPropertyAtom\"/>\n", level);

    // writing the Property
    //<swrl:propertyPredicate rdf:resource="http:/#hasInternationalNumber"/>
    writeString("<" + subfield_prop + " rdf:resource=\"" + ns_ + "#" + object_atom->object_property_expression->value() + "\"/>\n", level);

    // writing the Variable 1
    tmp = "<" + subfield_var + "1 rdf:resource=";
    if(object_atom->individual_involved_1 == nullptr)
      tmp += "\"urn:swrl:var#" + object_atom->var1;
    else
      tmp += "\"" + ns_ + "#" + object_atom->individual_involved_1->value();

    tmp += "\"/>\n";
    writeString(tmp, level);

    // writing the Variable 2
    tmp = "<" + subfield_var + "2 rdf:resource=";
    if(object_atom->individual_involved_2 == nullptr)
      tmp += "\"urn:swrl:var#" + object_atom->var2;
    else
      tmp += "\"" + ns_ + "#" + object_atom->individual_involved_2->value();

    tmp += "\"/>\n";
    writeString(tmp, level);
  }

  void RuleOwlWriter::writeAtom(DataPropertyAtom* data_atom, size_t level)
  {
    std::string field, tmp, subfield_prop, subfield_var;
    field = "rdf:Description";
    subfield_prop = "swrl:propertyPredicate";
    subfield_var = "swrl:argument";

    // writing the Atom type
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#DatavaluedPropertyAtom\"/>\n", level);

    // writing the Property
    //<swrl:propertyPredicate rdf:resource="http:/#hasInternationalNumber"/>
    writeString("<" + subfield_prop + " rdf:resource=\"" + ns_ + "#" + data_atom->data_property_expression->value() + "\"/>\n", level);

    // writing the Variable 1
    tmp = "<" + subfield_var + "1 rdf:resource=";
    if(data_atom->individual_involved == nullptr)
      tmp += "\"urn:swrl:var#" + data_atom->var1;
    else
      tmp += "\"" + ns_ + "#" + data_atom->individual_involved->value();

    tmp += "\"/>\n";
    writeString(tmp, level);

    // writing the Variable 2
    tmp = "<" + subfield_var + "2";
    if(data_atom->datatype_involved == nullptr)
      tmp += " rdf:resource=\"urn:swrl:var#" + data_atom->var2 + "\"/>\n";
    else
      tmp += " rdf:datatype=\"" + data_atom->datatype_involved->getNs() + "#" + data_atom->datatype_involved->type_ + "\">" +
             data_atom->datatype_involved->value_ + "</" + subfield_var + "2>\n";
    // <swrl:argument2 rdf:datatype="http://www.w3.org/2001/XMLSchema#boolean">true</swrl:argument2>
    writeString(tmp, level);
  }

  void RuleOwlWriter::writeAtom(BuiltinAtom* builtin_atom, size_t level)
  {
    (void)builtin_atom;
    (void)level;
  }

  void RuleOwlWriter::writeEquivalentClass(ClassBranch* branch)
  {
    AnonymousClassBranch* equiv = branch->equiv_relations_;

    if(equiv != nullptr)
    {
      for(auto* elem : equiv->ano_elems_)
      {
        std::string tmp, field;
        field = "owl:equivalentClass";
        const size_t level = 2;

        // single expression
        if(elem->sub_elements_.empty() &&
           elem->class_involved_ != nullptr &&
           elem->object_property_involved_ == nullptr)
        {
          writeString("<" + field + " " + getResource(elem) + "/>\n", level);
        }
        // Collection of expressions
        else
        {
          writeString("<" + field + ">\n", level);

          if(elem->logical_type_ != logical_none || elem->oneof == true)
            writeClassExpression(elem, level + 1);
          else
            writeRestriction(elem, level + 1);

          writeString("</" + field + ">\n", level);
        }
      }
    }
  }

  void RuleOwlWriter::writeRestriction(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp, field, subfield;

    field = "owl:Restriction";
    subfield = "owl:onProperty";

    writeString("<" + field + ">\n", level);

    // Property
    tmp = "<" + subfield + " " + getResource(ano_elem, "rdf:resource", true) + "/>\n";
    writeString(tmp, level + 1);

    // Cardinality
    if(ano_elem->card_.card_type_ == cardinality_max ||
       ano_elem->card_.card_type_ == cardinality_min ||
       ano_elem->card_.card_type_ == cardinality_exactly)
    {
      writeCardinalityValue(ano_elem, level + 1);
    }
    else if(ano_elem->card_.card_type_ == cardinality_only ||
            ano_elem->card_.card_type_ == cardinality_some ||
            ano_elem->card_.card_type_ == cardinality_value)
    {
      if(ano_elem->data_property_involved_ != nullptr)
        writeCardinalityRange(ano_elem, level + 1, true);
      else
        writeCardinalityRange(ano_elem, level + 1, false);
    }

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeClassExpression(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp, field;

    field = "owl:Class";

    writeString("<" + field + ">\n", level);

    if(ano_elem->logical_type_ == logical_or)
      writeUnion(ano_elem, level + 1);
    else if(ano_elem->logical_type_ == logical_and)
      writeIntersection(ano_elem, level + 1);
    else if(ano_elem->logical_type_ == logical_not)
      writeComplement(ano_elem, level + 1);
    else if(ano_elem->oneof == true)
      writeOneOf(ano_elem, level + 1);

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeDatatypeExpression(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string field;

    field = "rdfs:Datatype";

    writeString("<" + field + ">\n", level);

    if(ano_elem->logical_type_ == logical_or)
      writeUnion(ano_elem, level + 1, true);
    else if(ano_elem->logical_type_ == logical_and)
      writeIntersection(ano_elem, level + 1, true);
    else if(ano_elem->logical_type_ == logical_not)
      writeDataComplement(ano_elem, level + 1);

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeIntersection(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop)
  {
    std::string field;
    field = "owl:intersectionOf";

    writeString("<" + field + " " + "rdf:parseType=\"Collection\">\n", level);

    for(auto* child : ano_elem->sub_elements_)
      writeComplexDescription(child, level + 1, is_data_prop);

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeUnion(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop)
  {
    const std::string field = "owl:unionOf";

    writeString("<" + field + " " + "rdf:parseType=\"Collection\">\n", level);

    for(auto* child : ano_elem->sub_elements_)
      writeComplexDescription(child, level + 1, is_data_prop);

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeOneOf(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp;
    const std::string field = "owl:oneOf";

    writeString("<" + field + " " + " rdf:parseType=\"Collection\">\n", level);

    for(auto* child : ano_elem->sub_elements_)
    {
      tmp = "<rdf:Description " + getResource(child, "rdf:about") + "/>\n";
      writeString(tmp, level + 1);
    }

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeComplement(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp;
    const std::string field = "owl:complementOf";

    if(ano_elem->sub_elements_.front()->class_involved_ != nullptr && ano_elem->object_property_involved_ == nullptr)
    {
      tmp = "<" + field + " " + getResource(ano_elem->sub_elements_.front()) + "/>\n";
      writeString(tmp, level);
    }
    else
    {
      writeString("<" + field + ">\n", level);

      writeComplexDescription(ano_elem->sub_elements_.front(), level + 1);

      writeString("</" + field + ">\n", level);
    }
  }

  void RuleOwlWriter::writeDataComplement(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp;
    const std::string field = "owl:datatypeComplementOf";

    if(ano_elem->sub_elements_.front()->card_.card_range_ != nullptr)
    {
      tmp = "<" + field + " " + getResource(ano_elem->sub_elements_.front()) + "/>\n";
      writeString(tmp, level);
    }
    else
    {
      writeString("<" + field + ">\n", level);

      writeDatatypeExpression(ano_elem->sub_elements_.front(), level + 1);

      writeString("</" + field + ">\n", level);
    }
  }

  void RuleOwlWriter::writeComplexDescription(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop)
  {
    std::string tmp;

    if(ano_elem->sub_elements_.empty())
    {
      if(ano_elem->object_property_involved_ == nullptr && ano_elem->data_property_involved_ == nullptr)
      {
        tmp = "<rdf:Description " + getResource(ano_elem, "rdf:about") + "/>\n";
        writeString(tmp, level);
      }
      else
        writeRestriction(ano_elem, level);
    }
    else if(ano_elem->card_.card_type_ == cardinality_none)
    {
      if(is_data_prop)
        writeDatatypeExpression(ano_elem, level);
      else
        writeClassExpression(ano_elem, level);
    }
    else
      writeRestriction(ano_elem, level);
  }

  void RuleOwlWriter::writeCardinalityValue(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp, field;

    switch(ano_elem->card_.card_type_)
    {
    case cardinality_none:
      break;
    case cardinality_exactly:
      field = "owl:qualifiedCardinality";
      break;
    case cardinality_min:
      field = "owl:minQualifiedCardinality";
      break;
    case cardinality_max:
      field = "owl:maxQualifiedCardinality";
      break;
    case cardinality_error:
      Display::error("cardinality type error");
      break;
    default:
      Display::error("cardinality type " + std::to_string(ano_elem->card_.card_type_) + " not supported by this function");
      break;
    }

    tmp = "<" + field + " rdf:datatype=\"http://www.w3.org/2001/XMLSchema#nonNegativeInteger\">" +
          std::to_string(ano_elem->card_.card_number_) + "</" + field + ">\n";
    writeString(tmp, level);

    writeCardinality(ano_elem, level);
  }

  void RuleOwlWriter::writeCardinalityRange(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop)
  {
    std::string tmp, field;

    switch(ano_elem->card_.card_type_)
    {
    case cardinality_value:
      field = "owl:hasValue";
      if(is_data_prop)
      {
        tmp += "<" + field + " rdf:datatype=\"";
        tmp += ano_elem->card_.card_range_->getNs() + "#" + ano_elem->card_.card_range_->type_ + "\">" + ano_elem->card_.card_range_->value_;
        tmp += "</" + field + ">\n";
        std::cout << tmp << std::endl;
      }
      else
        tmp += "<" + field + " rdf:resource=\"" + ns_ + "#" + ano_elem->individual_involved_->value() + "\"/>\n";
      writeString(tmp, level);
      return;
    case cardinality_only:
      field = "owl:allValuesFrom";
      break;
    case cardinality_some:
      field = "owl:someValuesFrom";
      break;
    case cardinality_error:
      Display::error("cardinality type error");
      break;
    default:
      Display::error("cardinality type " + std::to_string(ano_elem->card_.card_type_) + " not supported by this function");
      break;
    }

    if(ano_elem->is_complex == false)
    {
      tmp = "<" + field + " " + getResource(ano_elem) + "/>\n";
      writeString(tmp, level);
    }
    else
    {
      writeString("<" + field + ">\n", level);

      if(is_data_prop == true)
        writeDatatypeExpression(ano_elem->sub_elements_.front(), level + 1);
      else
        writeComplexDescription(ano_elem->sub_elements_.front(), level + 1);

      writeString("</" + field + ">\n", level);
    }
  }

  void RuleOwlWriter::writeCardinality(AnonymousClassElement* ano_element, size_t level)
  {
    std::string tmp, field;

    if(ano_element->data_property_involved_ == nullptr)
    {
      field = "owl:onClass";

      if(ano_element->class_involved_ != nullptr)
      {
        tmp = "<" + field + " " + getResource(ano_element) + "/>\n";
        writeString(tmp, level);
      }
      else
      {
        writeString("<" + field + ">\n", level);

        writeClassExpression(ano_element->sub_elements_.front(), level + 1);

        writeString("</" + field + ">\n", level);
      }
    }
    else
    {
      field = "owl:onDataRange";

      if(ano_element->card_.card_range_ != nullptr)
      {
        tmp = "<" + field + " " + getResource(ano_element) + "/>\n";
        writeString(tmp, level);
      }
      else
      {
        writeString("<" + field + ">\n", level);

        writeDatatypeExpression(ano_element->sub_elements_.front(), level + 1);

        writeString("</" + field + ">\n", level);
      }
    }
  }

  std::string RuleOwlWriter::getResource(AnonymousClassElement* ano_elem, const std::string& attribute_name, bool used_property)
  {
    if(used_property == true)
    {
      if(ano_elem->object_property_involved_ != nullptr)
        return attribute_name + "=\"" + ns_ + "#" + ano_elem->object_property_involved_->value() + "\"";
      else if(ano_elem->data_property_involved_ != nullptr)
        return attribute_name + "=\"" + ns_ + "#" + ano_elem->data_property_involved_->value() + "\"";
      else
        return "";
    }
    else if(ano_elem->class_involved_ != nullptr)
      return attribute_name + "=\"" + ns_ + "#" + ano_elem->class_involved_->value() + "\"";

    else if(ano_elem->card_.card_range_ != nullptr)
      return attribute_name + "=\"" + ano_elem->card_.card_range_->getNs() + "#" + ano_elem->card_.card_range_->type_ + "\"";

    else if(ano_elem->individual_involved_ != nullptr)
      return attribute_name + "=\"" + ns_ + "#" + ano_elem->individual_involved_->value() + "\"";
    else
      return "";
  }

} // namespace ontologenius