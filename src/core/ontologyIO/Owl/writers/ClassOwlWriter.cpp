#include "ontologenius/core/ontologyIO/Owl/writers/ClassOwlWriter.h"

#include <algorithm>

#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"


namespace ontologenius {

ClassOwlWriter::ClassOwlWriter(ClassGraph* class_graph, AnonymousClassGraph* anonymous_graph, const std::string& ns)
{
  class_graph_ = class_graph;
  anonymous_graph_ = anonymous_graph;
  ns_ = ns;
}

void ClassOwlWriter::write(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

  std::vector<ClassBranch_t*> classes = class_graph_->get();
  

  for(auto& classe : classes)
    writeClass(classe);

  file_ = nullptr;
}

void ClassOwlWriter::writeGeneralAxioms(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

  std::vector<ClassBranch_t*> classes = class_graph_->get();
  writeDisjointWith(classes);

  file_ = nullptr;
}


void ClassOwlWriter::writeClass(ClassBranch_t* branch)
{
  std::string tmp = "    <!-- " + ns_ + "#" + branch->value() + " -->\n\n\
    <owl:Class rdf:about=\"" + ns_ + "#" + branch->value() + "\">\n";
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

void ClassOwlWriter::writeEquivalentClass(ClassBranch_t* branch)
{
  std::vector<AnonymousClassBranch_t*> equiv = anonymous_graph_->get();

  for(auto& elem : equiv)
  {
    if(elem->class_equiv_->value() == branch->value())
    {
      std::string start = "        <owl:equivalentClass";
      std::string end = "        </owl:equivalentClass>\n";
      std::string tmp;
      writeString(start);
      // single expression
      if(elem->ano_elem_->sub_elements_.size() == 0)
      {
        // Subclass only
        if(elem->ano_elem_->class_involved_ != nullptr && elem->ano_elem_->object_property_involved_ == nullptr)
        {
          tmp = " rdf:resource=\"" + ns_ + "#" + elem->ano_elem_->class_involved_->value();
          end = "\"/>\n";     
          writeString(tmp);
        }
        //Class expression
        else{
          std::string tmp = ">\n";
          writeString(tmp);
          writeRestriction(elem->ano_elem_, 12);
        }
      }
      // Collection of expressions
      else
      {
        std::string tmp = ">\n";
        writeString(tmp);
        writeCollection(elem->ano_elem_, 8);
      }
      writeString(end);
    }
  }

}

void ClassOwlWriter::writeCollection(AnonymousClassElement_t* ano_elem, int nb_ident)
{
  nb_ident +=  4;
  std::string indent(nb_ident, ' ');
  std::string indent_sub(nb_ident + 4, ' ');
  std::string start_class = "<owl:Class>\n";
  std::string end_class = "</owl:Class>\n";
  std::string start, end, tmp;

  if(ano_elem->nb_sub > 0)
  {
    if(ano_elem->andor == true)
    {
      start = "<owl:intersectionOf rdf:parseType=\"Collection\">\n";
      end = "</owl:intersectionOf>\n";

      writeString(indent + start_class);
      writeString(indent_sub + start);

      for(auto sub_elem : ano_elem->sub_elements_)
        writeCollection(sub_elem, nb_ident + 4);

      writeString(indent_sub + end);
      writeString(indent + end_class);
    }
    else if(ano_elem->andor == false)
    {
      start = "<owl:unionOf rdf:parseType=\"Collection\">\n";
      end = "</owl:unionOf>\n";

      writeString(indent + start_class);
      writeString(indent_sub + start);

      for(auto sub_elem : ano_elem->sub_elements_)
        writeCollection(sub_elem, nb_ident + 4);

      writeString(indent_sub + end);
      writeString(indent + end_class);
    }
  }
  else
  {
    if(ano_elem->class_involved_ != nullptr && ano_elem->object_property_involved_ == nullptr)
    {
      tmp = "<rdf:Description rdf:about=\"" + ns_ + "#" + ano_elem->class_involved_->value() + "\"/>\n"; 
      writeString(indent + tmp);
    }
    else{
      writeRestriction(ano_elem, nb_ident);
    }
  }
}

void ClassOwlWriter::writeRestriction(AnonymousClassElement_t* ano_element, int nb_indent)
{
  std::string indent(nb_indent, ' ');
  std::string indent_sub(nb_indent + 4, ' ');
  std::string start = indent + "<owl:Restriction>\n";
  std::string end = indent + "</owl:Restriction>\n";
  std::string tmp;

  tmp = indent_sub + "<owl:onProperty rdf:resource=\"" + ns_ + "#";

  if(ano_element->data_property_involved_ == nullptr)
  {
    tmp += ano_element->object_property_involved_->value() + "\"/>\n";
  }
  else{
    tmp += ano_element->data_property_involved_->value() + "\"/>\n";
  }

  tmp += indent_sub;
  switch(ano_element->card_.card_type_)
  {
    
    case value_ : 
      tmp += "<owl:hasValue rdf:resource=\"" + ns_ + "#" + ano_element->individual_involved_->value() + "\"/>\n";
      break;
    case only_ : 
      tmp += "<owl:allValuesFrom rdf:resource=\"" + ns_ + "#" + ano_element->class_involved_->value() + "\"/>\n";
      break;
    case exactly_ : 
      tmp += "<owl:qualifiedCardinality rdf:datatype=\"http://www.w3.org/2001/XMLSchema#nonNegativeInteger\">" 
            + std::to_string(ano_element->card_.card_number_) +
            "</owl:qualifiedCardinality>" + "\n";
      tmp+= indent_sub;
      tmp += writeCardinality(ano_element);
      break;
    case min_ : 
      tmp += "<owl:minQualifiedCardinality rdf:datatype=\"http://www.w3.org/2001/XMLSchema#nonNegativeInteger\">" 
            + std::to_string(ano_element->card_.card_number_) +
            "</owl:minQualifiedCardinality>" + "\n";
      tmp+= indent_sub;
      tmp += writeCardinality(ano_element);
      break;
    case max_ : 
      tmp += "<owl:maxQualifiedCardinality rdf:datatype=\"http://www.w3.org/2001/XMLSchema#nonNegativeInteger\">" 
            + std::to_string(ano_element->card_.card_number_) +
            "</owl:maxQualifiedCardinality>" + "\n";
      tmp+= indent_sub;
      tmp += writeCardinality(ano_element);
      break;
   
    case some_ :
      if(ano_element->data_property_involved_ == nullptr)
        tmp += "<owl:someValuesFrom rdf:resource=\"" + ns_ + "#" + ano_element->class_involved_->value() + "\"/>\n";
      else
        tmp += "<owl:someValuesFrom rdf:datatype=\"" + ano_element->card_.card_range_->getNs() + "#" + ano_element->card_.card_range_->type_ + "\"/>\n";
      break;
  }

  writeString(start);
  writeString(tmp);
  writeString(end);

}

std::string ClassOwlWriter::writeCardinality(AnonymousClassElement_t* ano_element)
{
  std::string tmp;
  if( ano_element->data_property_involved_ == nullptr )
    tmp = "<owl:onClass rdf:resource=\"" + ns_ + "#" + ano_element->class_involved_->value() + "\"/>\n";
  else
    tmp = "<owl:onDataRange rdf:resource=\"" + ano_element->card_.card_range_->getNs() + "#" + ano_element->card_.card_range_->type_ + "\"/>\n";

  return tmp;
}

void ClassOwlWriter::writeSubClassOf(ClassBranch_t* branch)
{
  for(auto& mother : branch->mothers_)
    if(mother.infered == false)
    {
      std::string proba = (mother < 1.0) ? " onto:probability=\"" + std::to_string(mother.probability) + "\"" : "";
      std::string tmp = "        <rdfs:subClassOf" +
                        proba +
                        " rdf:resource=\"" + ns_ + "#" +
                        mother.elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

void ClassOwlWriter::writeDisjointWith(ClassBranch_t* branch)
{
  if(branch->disjoints_.size() < 2)
    for(auto& disjoint : branch->disjoints_)
      if(disjoint.infered == false)
      {
        std::string tmp = "        <owl:disjointWith" +
                          getProba(disjoint) +
                          " rdf:resource=\"" + ns_ + "#" +
                          disjoint.elem->value()
                          + "\"/>\n";
        writeString(tmp);
      }
}

void ClassOwlWriter::writeDisjointWith(std::vector<ClassBranch_t*>& classes)
{
  std::string start = "    <rdf:Description>\n\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDisjointClasses\"/>\n";

  std::string end = "    </rdf:Description>\n";

  std::set<std::set<ClassBranch_t*>> disjoints_vects;

  for(auto& classe : classes)
  {
    if(classe->disjoints_.size() > 1)
      getDisjointsSets(classe, disjoints_vects);
  }

  for(auto& disjoints_set : disjoints_vects)
  {
    std::string tmp;
    tmp += "        <owl:members rdf:parseType=\"Collection\">\n";

    for(auto& disj : disjoints_set)
    {
      tmp += "             <rdf:Description rdf:about=\"" + ns_ + "#" +
      disj->value() +
      "\"/>\n";
    }

    tmp += "        </owl:members>\n";
    if(disjoints_set.size() > 0)
    {
      writeString(start);
      writeString(tmp);
      writeString(end);
    }
  }
}

void ClassOwlWriter::getDisjointsSets(ClassBranch_t* base, std::set<std::set<ClassBranch_t*>>& res)
{
  std::set<ClassBranch_t*> restriction_set;

  for(auto& disjoint : base->disjoints_)
    restriction_set.insert(disjoint.elem);
  restriction_set.insert(base);

  for(auto& disjoint : base->disjoints_)
  {
    std::set<ClassBranch_t*> base_set;
    base_set.insert(base);
    base_set.insert(disjoint.elem);
    getDisjointsSets(disjoint.elem, base_set, restriction_set, res);
  }
}

void ClassOwlWriter::getDisjointsSets(ClassBranch_t* last, const std::set<ClassBranch_t*>& base_set, const std::set<ClassBranch_t*>& restriction_set, std::set<std::set<ClassBranch_t*>>& res)
{
  std::set<ClassBranch_t*> local_disjoints;
  for(auto& disjoint : last->disjoints_)
    local_disjoints.insert(disjoint.elem);
  std::vector<ClassBranch_t*> new_restriction_vect;
  std::set_intersection(restriction_set.begin(), restriction_set.end(), local_disjoints.begin(), local_disjoints.end(), std::back_inserter(new_restriction_vect));
  std::set<ClassBranch_t*> new_restriction_set;
  for(auto& it : new_restriction_vect)
    new_restriction_set.insert(it);

  bool leaf = true;
  for(auto& disjoint : last->disjoints_)
  {
    if(restriction_set.find(disjoint.elem) != restriction_set.end())
    {
      if(base_set.find(disjoint.elem) == base_set.end())
      {
        std::set<ClassBranch_t*> new_set = base_set;
        new_set.insert(disjoint.elem);
        getDisjointsSets(disjoint.elem, new_set, new_restriction_set, res);
        leaf = false;
      }
    }
  }

  if(leaf)
    res.insert(base_set);
}

void ClassOwlWriter::writeObjectProperties(ClassBranch_t* branch)
{
  for(ClassObjectRelationElement_t& relation : branch->object_relations_)
    if(relation.infered == false)
    {
      std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
      std::string tmp = "        <" +
                        relation.first->value() +
                        proba +
                        " rdf:resource=\"" + ns_ + "#" +
                        relation.second->value() +
                        "\"/>\n";
      writeString(tmp);
    }
}

void ClassOwlWriter::writeDataProperties(ClassBranch_t* branch)
{
  for(ClassDataRelationElement_t& relation : branch->data_relations_)
    if(relation.infered == false)
    {
      std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
      std::string tmp = "        <" +
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
