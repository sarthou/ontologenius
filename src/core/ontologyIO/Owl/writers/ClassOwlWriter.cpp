#include "ontologenius/core/ontologyIO/Owl/writers/ClassOwlWriter.h"

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <set>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

  ClassOwlWriter::ClassOwlWriter(ClassGraph* class_graph, const std::string& ns) : class_graph_(class_graph)
  {
    ns_ = ns;
  }

  void ClassOwlWriter::write(FILE* file)
  {
    file_ = file;

    const std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

    const std::vector<ClassBranch*> classes = class_graph_->get();

    for(auto* classe : classes)
      writeClass(classe);

    file_ = nullptr;
  }

  void ClassOwlWriter::writeGeneralAxioms(FILE* file)
  {
    file_ = file;

    const std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

    std::vector<ClassBranch*> classes = class_graph_->get();
    writeDisjointWith(classes);

    file_ = nullptr;
  }

  void ClassOwlWriter::writeClass(ClassBranch* branch)
  {
    std::string tmp = "    <!-- " + ns_ + "#" + branch->value() + " -->\n\n\
    <owl:Class rdf:about=\"" +
                      ns_ + "#" + branch->value() + "\">\n";
    writeString(tmp);

    writeEquivalentClass(branch);
    writeSubClassOf(branch);

    writeDisjointWith(branch);

    writeObjectProperties(branch);
    writeDataProperties(branch);

    writeDictionary(branch);
    writeMutedDictionary(branch);

    tmp = "    </owl:Class>\n\n\n\n";
    writeString(tmp);
  }

  void ClassOwlWriter::writeEquivalentClass(ClassBranch* branch)
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
          tmp = "<" + field + " " + getResource(elem) + "/>\n";
          writeString(tmp, level);
        }
        // Collection of expressions
        else
        {
          tmp = "<" + field + ">\n";
          writeString(tmp, level);

          if(elem->logical_type_ != logical_none || elem->oneof == true)
            writeClassExpression(elem, level + 1);
          else
            writeRestriction(elem, level + 1);

          tmp = "</" + field + ">\n";
          writeString(tmp, level);
        }
      }
    }
  }

  void ClassOwlWriter::writeRestriction(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp, field, subfield;

    field = "owl:Restriction";
    subfield = "owl:onProperty";

    tmp = "<" + field + ">\n";
    writeString(tmp, level);

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

    tmp = "</" + field + ">\n";
    writeString(tmp, level);
  }

  void ClassOwlWriter::writeClassExpression(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp, field;

    field = "owl:Class";

    tmp = "<" + field + ">\n";
    writeString(tmp, level);

    if(ano_elem->logical_type_ == logical_or)
      writeUnion(ano_elem, level + 1);
    else if(ano_elem->logical_type_ == logical_and)
      writeIntersection(ano_elem, level + 1);
    else if(ano_elem->logical_type_ == logical_not)
      writeComplement(ano_elem, level + 1);
    else if(ano_elem->oneof == true)
      writeOneOf(ano_elem, level + 1);

    tmp = "</" + field + ">\n";
    writeString(tmp, level);
  }

  void ClassOwlWriter::writeDatatypeExpression(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp, field;

    field = "rdfs:Datatype";

    tmp = "<" + field + ">\n";
    writeString(tmp, level);

    if(ano_elem->logical_type_ == logical_or)
      writeUnion(ano_elem, level + 1, true);
    else if(ano_elem->logical_type_ == logical_and)
      writeIntersection(ano_elem, level + 1, true);
    else if(ano_elem->logical_type_ == logical_not)
      writeDataComplement(ano_elem, level + 1);

    tmp = "</" + field + ">\n";
    writeString(tmp, level);
  }

  void ClassOwlWriter::writeIntersection(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop)
  {
    std::string tmp, field;
    field = "owl:intersectionOf";

    tmp = "<" + field + " " + "rdf:parseType=\"Collection\">\n";
    writeString(tmp, level);

    for(auto* child : ano_elem->sub_elements_)
      writeComplexDescription(child, level + 1, is_data_prop);

    tmp = "</" + field + ">\n";
    writeString(tmp, level);
  }

  void ClassOwlWriter::writeUnion(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop)
  {
    std::string tmp;
    const std::string field = "owl:unionOf";

    tmp = "<" + field + " " + "rdf:parseType=\"Collection\">\n";
    writeString(tmp, level);

    for(auto* child : ano_elem->sub_elements_)
      writeComplexDescription(child, level + 1, is_data_prop);

    tmp = "</" + field + ">\n";
    writeString(tmp, level);
  }

  void ClassOwlWriter::writeOneOf(AnonymousClassElement* ano_elem, size_t level)
  {
    std::string tmp;
    const std::string field = "owl:oneOf";

    tmp = "<" + field + " " + "rdf:parseType=\"Collection\">\n";
    writeString(tmp, level);

    for(auto* child : ano_elem->sub_elements_)
    {
      tmp = "<rdf:Description " + getResource(child, "rdf:about") + "/>\n";
      writeString(tmp, level + 1);
    }

    tmp = "</" + field + ">\n";
    writeString(tmp, level);
  }

  void ClassOwlWriter::writeComplement(AnonymousClassElement* ano_elem, size_t level)
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
      tmp = "<" + field + ">\n";
      writeString(tmp, level);

      writeComplexDescription(ano_elem->sub_elements_.front(), level + 1);

      tmp = "</" + field + ">\n";
      writeString(tmp, level);
    }
  }

  void ClassOwlWriter::writeDataComplement(AnonymousClassElement* ano_elem, size_t level)
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
      tmp = "<" + field + ">\n";
      writeString(tmp, level);

      writeDatatypeExpression(ano_elem->sub_elements_.front(), level + 1);

      tmp = "</" + field + ">\n";
      writeString(tmp, level);
    }
  }

  void ClassOwlWriter::writeComplexDescription(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop)
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

  void ClassOwlWriter::writeCardinalityValue(AnonymousClassElement* ano_elem, size_t level)
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

  void ClassOwlWriter::writeCardinalityRange(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop)
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
      tmp = "<" + field + ">\n";
      writeString(tmp, level);

      if(is_data_prop == true)
        writeDatatypeExpression(ano_elem->sub_elements_.front(), level + 1);
      else
        writeComplexDescription(ano_elem->sub_elements_.front(), level + 1);

      tmp = "</" + field + ">\n";
      writeString(tmp, level);
    }
  }

  void ClassOwlWriter::writeCardinality(AnonymousClassElement* ano_element, size_t level)
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
        tmp = "<" + field + ">\n";
        writeString(tmp, level);

        writeClassExpression(ano_element->sub_elements_.front(), level + 1);

        tmp = "</" + field + ">\n";
        writeString(tmp, level);
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
        tmp = "<" + field + ">\n";
        writeString(tmp, level);

        writeDatatypeExpression(ano_element->sub_elements_.front(), level + 1);

        tmp = "</" + field + ">\n";
        writeString(tmp, level);
      }
    }
  }

  std::string ClassOwlWriter::getResource(AnonymousClassElement* ano_elem, const std::string& attribute_name, bool used_property)
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

  void ClassOwlWriter::writeSubClassOf(ClassBranch* branch)
  {
    for(auto& mother : branch->mothers_)
      if(mother.infered == false)
      {
        const std::string proba = (mother < 1.0) ? " onto:probability=\"" + std::to_string(mother.probability) + "\"" : "";
        const std::string tmp = "        <rdfs:subClassOf" +
                                proba +
                                " rdf:resource=\"" + ns_ + "#" +
                                mother.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void ClassOwlWriter::writeDisjointWith(ClassBranch* branch)
  {
    if(branch->disjoints_.size() < 2)
      for(auto& disjoint : branch->disjoints_)
        if(disjoint.infered == false)
        {
          const std::string tmp = "        <owl:disjointWith" +
                                  getProba(disjoint) +
                                  " rdf:resource=\"" + ns_ + "#" +
                                  disjoint.elem->value() + "\"/>\n";
          writeString(tmp);
        }
  }

  void ClassOwlWriter::writeDisjointWith(std::vector<ClassBranch*>& classes)
  {
    const std::string start = "    <rdf:Description>\n\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDisjointClasses\"/>\n";

    const std::string end = "    </rdf:Description>\n";

    std::set<std::set<ClassBranch*>> disjoints_vects;

    for(auto& classe : classes)
    {
      if(classe->disjoints_.size() > 1)
        getDisjointsSets(classe, disjoints_vects);
    }

    for(const auto& disjoints_set : disjoints_vects)
    {
      std::string tmp;
      tmp += "        <owl:members rdf:parseType=\"Collection\">\n";

      for(const auto& disj : disjoints_set)
      {
        tmp += "             <rdf:Description rdf:about=\"" + ns_ + "#" +
               disj->value() +
               "\"/>\n";
      }

      tmp += "        </owl:members>\n";
      if(disjoints_set.empty() == false)
      {
        writeString(start);
        writeString(tmp);
        writeString(end);
      }
    }
  }

  void ClassOwlWriter::getDisjointsSets(ClassBranch* base, std::set<std::set<ClassBranch*>>& res)
  {
    std::set<ClassBranch*> restriction_set;

    for(auto& disjoint : base->disjoints_)
      restriction_set.insert(disjoint.elem);
    restriction_set.insert(base);

    for(auto& disjoint : base->disjoints_)
    {
      std::set<ClassBranch*> base_set;
      base_set.insert(base);
      base_set.insert(disjoint.elem);
      getDisjointsSets(disjoint.elem, base_set, restriction_set, res);
    }
  }

  void ClassOwlWriter::getDisjointsSets(ClassBranch* last, const std::set<ClassBranch*>& base_set, const std::set<ClassBranch*>& restriction_set, std::set<std::set<ClassBranch*>>& res)
  {
    std::set<ClassBranch*> local_disjoints;
    for(auto& disjoint : last->disjoints_)
      local_disjoints.insert(disjoint.elem);
    std::vector<ClassBranch*> new_restriction_vect;
    std::set_intersection(restriction_set.begin(), restriction_set.end(), local_disjoints.begin(), local_disjoints.end(), std::back_inserter(new_restriction_vect));
    std::set<ClassBranch*> new_restriction_set;
    for(auto& it : new_restriction_vect)
      new_restriction_set.insert(it);

    bool leaf = true;
    for(auto& disjoint : last->disjoints_)
    {
      if(restriction_set.find(disjoint.elem) != restriction_set.end())
      {
        if(base_set.find(disjoint.elem) == base_set.end())
        {
          std::set<ClassBranch*> new_set = base_set;
          new_set.insert(disjoint.elem);
          getDisjointsSets(disjoint.elem, new_set, new_restriction_set, res);
          leaf = false;
        }
      }
    }

    if(leaf)
      res.insert(base_set);
  }

  void ClassOwlWriter::writeObjectProperties(ClassBranch* branch)
  {
    for(const ClassObjectRelationElement& relation : branch->object_relations_)
      if(relation.infered == false)
      {
        const std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
        const std::string tmp = "        <" +
                                relation.first->value() +
                                proba +
                                " rdf:resource=\"" + ns_ + "#" +
                                relation.second->value() +
                                "\"/>\n";
        writeString(tmp);
      }
  }

  void ClassOwlWriter::writeDataProperties(ClassBranch* branch)
  {
    for(const ClassDataRelationElement& relation : branch->data_relations_)
      if(relation.infered == false)
      {
        const std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
        const std::string tmp = "        <" +
                                relation.first->value() +
                                proba +
                                " rdf:datatype=\"" +
                                relation.second->getNs() +
                                "#" +
                                relation.second->type_ +
                                "\">" +
                                relation.second->value_ +
                                "</" +
                                relation.first->value() +
                                ">\n";
        writeString(tmp);
      }
  }

} // namespace ontologenius
