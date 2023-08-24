#include "ontologenius/core/ontologyIO/Owl/OntologyOwlReader.h"

#include <fstream>

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"
#include "ontologenius/utils/String.h"

namespace ontologenius {

OntologyOwlReader::OntologyOwlReader(ClassGraph* class_graph,
                                     ObjectPropertyGraph* object_property_graph,
                                     DataPropertyGraph* data_property_graph,
                                     IndividualGraph* individual_graph,
                                     AnonymousClassGraph* anonymous_graph) :
                                                      OntologyReader(class_graph, object_property_graph, data_property_graph, individual_graph, anonymous_graph),
                                                      card_map_{{ "owl:someValuesFrom", "some" },
                                                                { "owl:allValuesFrom", "only" },
                                                                { "owl:minQualifiedCardinality", "min" },
                                                                { "owl:maxQualifiedCardinality", "max" },
                                                                { "owl:qualifiedCardinality", "exactly" },
                                                                { "owl:hasValue", "value" }}
{}

OntologyOwlReader::OntologyOwlReader(Ontology& onto) : OntologyReader(onto),
                                                       card_map_{{ "owl:someValuesFrom", "some" },
                                                                 { "owl:allValuesFrom", "only" },
                                                                 { "owl:minQualifiedCardinality", "min" },
                                                                 { "owl:maxQualifiedCardinality", "max" },
                                                                 { "owl:qualifiedCardinality", "exactly" },
                                                                 { "owl:hasValue", "value" }}
{}
int OntologyOwlReader::readFromUri(std::string content, const std::string& uri, bool individual)
{
  removeDocType(content);

  TiXmlDocument doc;
  doc.Parse((const char*)content.c_str(), nullptr, TIXML_ENCODING_UTF8);
  TiXmlElement* rdf = doc.FirstChildElement();
  if(individual == false)
    return read(rdf, uri);
  else
    return readIndividual(rdf, uri);
}

int OntologyOwlReader::readFromFile(const std::string& file_name, bool individual)
{
  std::string response = "";
  std::string tmp = "";
  std::ifstream f(file_name);

  if(!f.is_open())
  {
    Display::error("Fail to open : " + file_name);
    return -1;
  }

  while(getline(f,tmp))
  {
    response += tmp;
  }
  removeDocType(response);

  TiXmlDocument doc;
  doc.Parse((const char*)response.c_str(), nullptr, TIXML_ENCODING_UTF8);
  TiXmlElement* rdf = doc.FirstChildElement();
  if(individual == false)
    return read(rdf, file_name);
  else
    return readIndividual(rdf, file_name);
}

std::vector<std::string> OntologyOwlReader::getImportsFromRaw(std::string content)
{
  std::vector<std::string> imports;
  removeDocType(content);

  TiXmlDocument doc;
  doc.Parse((const char*)content.c_str(), nullptr, TIXML_ENCODING_UTF8);
  TiXmlElement* rdf = doc.FirstChildElement();

  if(rdf == nullptr)
    return {};
  else if(std::string(rdf->Value()) != "rdf:RDF")
    return {};
  else
  {
    auto ontology_elem = rdf->FirstChildElement("owl:Ontology");
    for(TiXmlElement* elem = ontology_elem->FirstChildElement("owl:imports"); elem != nullptr; elem = elem->NextSiblingElement("owl:imports"))
      imports.emplace_back(elem->Attribute("rdf:resource"));
  }

  return imports;
}

std::vector<std::string> OntologyOwlReader::getImportsFromFile(const std::string& file_name)
{
  std::string raw_file = "";
  std::string tmp = "";
  std::ifstream f(file_name);

  if(!f.is_open())
    return {};

  while(getline(f,tmp))
    raw_file += tmp;
  
  return getImportsFromRaw(raw_file);
}

int OntologyOwlReader::read(TiXmlElement* rdf, const std::string& name)
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
      std::cout << "/ inverse      | * type    | " << std::endl;
      std::cout << "$ has property | ^ related | " << std::endl;
      std::cout << "************************************" << std::endl;
    }

    std::vector<TiXmlElement*> elem_classes, elem_descriptions, elem_obj_prop, elem_data_prop, elem_annotation_prop;
    std::string elem_name;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != nullptr; elem = elem->NextSiblingElement())
    {
      elem_name = elem->Value();
      if(elem_name == "owl:Class")
        elem_classes.push_back(elem);
      else if(elem_name == "rdf:Description")
        elem_descriptions.push_back(elem);
      else if(elem_name == "owl:ObjectProperty")
        elem_obj_prop.push_back(elem);
      else if(elem_name == "owl:DatatypeProperty")
        elem_data_prop.push_back(elem);
      else if(elem_name == "owl:AnnotationProperty")
        elem_annotation_prop.push_back(elem);
    }

    if(display_)
      std::cout << "├── Class" << std::endl;
    for(TiXmlElement* elem : elem_classes)
      readClass(elem);
    if(display_)
      std::cout << "├── Description" << std::endl;
    for(TiXmlElement* elem : elem_descriptions)
      readDescription(elem);
    if(display_)
      std::cout << "├── Object property" << std::endl;
    for(TiXmlElement* elem : elem_obj_prop)
      readObjectProperty(elem);
    if(display_)
      std::cout << "├── Data property" << std::endl;
    for(TiXmlElement* elem : elem_data_prop)
      readDataProperty(elem);
    if(display_)
      std::cout << "├── Annotation property" << std::endl;
    for(TiXmlElement* elem : elem_annotation_prop)
      readAnnotationProperty(elem);
    if(display_)
      std::cout << "└── "<< nb_loaded_elem_ << " readed ! " << std::endl;
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

int OntologyOwlReader::readIndividual(TiXmlElement* rdf, const std::string& name)
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
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != nullptr; elem = elem->NextSiblingElement())
      readIndividual(elem);
    if(display_)
      std::cout << "├── Description" << std::endl;
    for(TiXmlElement* elem = rdf->FirstChildElement(); elem != nullptr; elem = elem->NextSiblingElement())
      readIndividualDescription(elem);
    if(display_)
      std::cout << "└── "<< nb_loaded_elem_ << " readed ! " << std::endl;
    return NO_ERROR;
  }
}

void OntologyOwlReader::readClass(TiXmlElement* elem)
{
  std::string node_name = "";
  ObjectVectors_t object_vector;
  const char* attr = elem->Attribute("rdf:about");
  if(attr != nullptr)
  {
    node_name = getName(std::string(attr));
    if(display_)
      std::cout << "│   ├──" << node_name << std::endl;
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      std::string sub_elem_name = sub_elem->Value();

      float probability = getProbability(sub_elem);

      if(sub_elem_name == "rdfs:subClassOf")
        push(object_vector.mothers_, sub_elem, probability, "+");
      else if(sub_elem_name == "owl:disjointWith")
        push(object_vector.disjoints_, sub_elem, probability, "-");
      else if(sub_elem_name == "rdfs:label")
        pushLang(object_vector.dictionary_, sub_elem);
      else if(sub_elem_name == "onto:label")
        pushLang(object_vector.muted_dictionary_, sub_elem);
      else if(sub_elem_name == "owl:equivalentClass")
      {
        AnonymousClassVectors_t ano_class = readEquivalentClass(sub_elem, attr);
        anonymous_graph_->add(node_name, ano_class);
        push(object_vector.equivalences_, ano_class.str_equivalences , "=");
      }
      else
      {
        std::string ns = sub_elem_name.substr(0,sub_elem_name.find(':'));
        if((ns != "owl") && (ns != "rdf") && (ns != "rdfs"))
        {
          std::string property = sub_elem_name.substr(sub_elem_name.find(':')+1);
          if(testAttribute(sub_elem, "rdf:resource"))
            OntologyReader::push(object_vector.object_relations_, Pair_t<std::string, std::string>(property, toString(sub_elem), probability), "$", "^");
          else if(testAttribute(sub_elem, "rdf:datatype"))
          {
            const char* value = sub_elem->GetText();
            if(value != nullptr)
            {
              LiteralNode data(toString(sub_elem, "rdf:datatype"), std::string(value));
              OntologyReader::push(object_vector.data_relations_, Pair_t<std::string, std::string>(property, data.value(), probability), "$", "^");
            }
          }
        }
      }
    }
  }
  class_graph_->add(node_name, object_vector);
  nb_loaded_elem_++;
}

AnonymousClassVectors_t OntologyOwlReader::readEquivalentClass(TiXmlElement* elem, const std::string& class_name)
{
  AnonymousClassVectors_t ano;
  ano.class_equiv = class_name;
  std::vector<std::string> vect;
  //std::cout << "New anonymous class with " << class_name << std::endl;
  if(elem->FirstChild() == nullptr)
  {
      ExpressionMember_t* exp = new ExpressionMember_t;
      exp->mother = nullptr;
      exp->rest.restriction_range = getName(elem->Attribute("rdf:resource"));
      exp->str_equivalence = exp->rest.getRestriction();
      ano.equivalence = exp;
      ano.str_equivalences = exp->str_equivalence;

      vect.push_back(exp->rest.getRestriction());
      ano.equiv_vect.push_back(vect);
  }
 
  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    std::string sub_elem_name = sub_elem->Value();
    
    ExpressionMember_t* exp = new ExpressionMember_t();
    exp->mother = nullptr;
    ano.equivalence = exp;

    if(sub_elem_name == "owl:Restriction")
    {
      ExpressionMember_t* exp2 = new ExpressionMember_t();
      exp2->mother = exp;
      exp->intersects.push_back(exp2);
      readRestriction(sub_elem, exp2, ano);
      exp->str_equivalence = exp2->str_equivalence;
      ano.str_equivalences = exp->str_equivalence;
      ano.equiv_vect.push_back(exp->rest.getRestrictionVector());
    }
    else
    {
      readCollection(sub_elem, exp, ano);
      ano.str_equivalences = exp->str_equivalence;
    }
  }

  return ano; 
}

void OntologyOwlReader::readCollection(TiXmlElement* elem, ExpressionMember_t* exp, AnonymousClassVectors_t& ano)
{

  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    if(getName(sub_elem->Value())  == "owl:Class" || getName(sub_elem->Value())  == "rdfs:Datatype")
      readCollection(sub_elem, exp, ano);
    else
    {
      ExpressionMember_t* exp2 = new ExpressionMember_t();
      exp2->mother = exp;
      exp2->andor = false;
      exp->intersects.push_back(exp2);

      if(getName(sub_elem->Value()) == "owl:intersectionOf")
      {
        exp2->andor = true;
        exp2->nb_sub = getNbChildren(sub_elem);
        readCollection(sub_elem, exp2, ano);
        exp2->UpdateEquiv();
        exp->str_equivalence = exp2->str_equivalence;
      }
      else if(getName(sub_elem->Value())  == "owl:unionOf")
      {
        exp2->andor = false;
        exp2->nb_sub = getNbChildren(sub_elem);
        readCollection(sub_elem, exp2, ano);
        exp2->UpdateEquiv();
        exp->str_equivalence = exp2->str_equivalence;
      }
      else if(getName(sub_elem->Value())  == "owl:Restriction")
      {
        readRestriction(sub_elem, exp2, ano);
      }
      else if(getName(sub_elem->Value())  == "rdf:Description")
      {
        std::string s = sub_elem->Attribute("rdf:about");
        if(isIn("http://www.w3.org/", s)){
          exp2->rest.restriction_range = s;
          updatePropertyType(exp2);
        }   
        else
          exp2->rest.restriction_range = getName(s);
        exp2->str_equivalence = exp2->rest.getRestriction();
      }
       else if(getName(sub_elem->Value())  == "owl:complementOf" || getName(sub_elem->Value())  == "owl:datatypeComplementOf")
      {
        exp2->andor = false;
        exp2->negation = true;
        exp2->nb_sub = 1;

        if(sub_elem->FirstChildElement() == nullptr)
        {
          ExpressionMember_t* exp3 = new ExpressionMember_t;
          exp2->intersects.push_back(exp3);
          exp3->mother = exp2;

          std::string s = sub_elem->Attribute("rdf:resource");
          if(isIn("http://www.w3.org/", s))
          {
            exp3->rest.card.cardinality_range = s;
            updatePropertyType(exp3);
          }
          else
            exp3->rest.card.cardinality_range = getName(s);
          exp3->str_equivalence = exp3->rest.getRestriction();
          exp2->str_equivalence = " not (" + exp3->str_equivalence + ")";
        }
        else{
          readCollection(sub_elem, exp2, ano);
          exp2->UpdateEquiv();
        }
        exp->str_equivalence = exp2->str_equivalence;

      }
    }
  }
}

void OntologyOwlReader::readRestriction(TiXmlElement* elem, ExpressionMember_t* exp, AnonymousClassVectors_t& ano)
{

  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    std::string sub_elem_name = sub_elem->Value();
    
    if(sub_elem_name == "owl:onProperty")
      exp->rest.property = getName(sub_elem->Attribute("rdf:resource"));
    else if(sub_elem_name == "owl:onClass" || sub_elem_name == "owl:onDataRange")
    {
      const char* resource = sub_elem->Attribute("rdf:resource");
      if(resource != nullptr)
      {
        std::string attr_class = sub_elem->Attribute("rdf:resource");
        if(isIn("http://www.w3.org/", attr_class))
        {
          exp->rest.restriction_range = attr_class;
          updatePropertyType(exp);
        }
        else
          exp->rest.restriction_range= getName(attr_class);
      }
      else
      {
        exp->nb_sub += 1;
        readCollection(sub_elem, exp, ano);
      }
    }
    else
    {
      readCardinality(sub_elem, exp);
      // someValuesFrom with empty rdf resource
      if(sub_elem->FirstChildElement() != nullptr)
      {
        exp->nb_sub += 1;
        readCollection(sub_elem, exp, ano);
      }
    }
    if(exp->nb_sub == 1)
      exp->str_equivalence = "(" + exp->rest.getRestriction() + exp->intersects.front()->str_equivalence + ")";
    else
      exp->str_equivalence = "(" + exp->rest.getRestriction() + ")";
  }

}

void OntologyOwlReader::readCardinality(TiXmlElement* elem, ExpressionMember_t* exp)
{
  std::string sub_elem_name = elem->Value();

  const char* resource = elem->Attribute("rdf:resource");
  exp->rest.card.cardinality_type = card_map_[sub_elem_name];

  if(elem->GetText() != nullptr)
    exp->rest.card.cardinality_number = elem->GetText();

  if(resource != nullptr)
  {
    std::string s = resource;
    if(isIn("http://www.w3.org/", s))
    {
      exp->rest.card.cardinality_range = resource;
      updatePropertyType(exp);
    }
    else
      exp->rest.card.cardinality_range = getName(resource);
  }
}

void OntologyOwlReader::updatePropertyType(ExpressionMember_t* exp)
{
  if(!exp->rest.property.empty())
  {
    exp->isDataProp = true;
    return;
  }
  else{
    if(exp->mother != nullptr)
      updatePropertyType(exp->mother);
    else
      return;
  }
}

void OntologyOwlReader::readIndividual(TiXmlElement* elem)
{
  std::string elem_name = elem->Value();
  if(elem_name == "owl:NamedIndividual")
  {
    std::string node_name = "";
    IndividualVectors_t individual_vector;
    const char* attr = elem->Attribute("rdf:about");
    if(attr != nullptr)
    {
      node_name = getName(std::string(attr));
      if(display_)
        std::cout << "│   ├──" << node_name << std::endl;
      for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      {
        std::string sub_elem_name = sub_elem->Value();
        float probability = getProbability(sub_elem);

        if(sub_elem_name == "rdf:type")
          push(individual_vector.is_a_, sub_elem, probability, "+");
        else if(sub_elem_name == "owl:sameAs")
          push(individual_vector.same_as_, sub_elem, probability, "=");
        else if(sub_elem_name == "rdfs:label")
          pushLang(individual_vector.dictionary_, sub_elem);
        else if(sub_elem_name == "onto:label")
          pushLang(individual_vector.muted_dictionary_, sub_elem);
        else
        {
          std::string ns = sub_elem_name.substr(0,sub_elem_name.find(':'));
          if((ns != "owl") && (ns != "rdf") && (ns != "rdfs"))
          {
            std::string property = sub_elem_name.substr(sub_elem_name.find(':')+1);
            if(testAttribute(sub_elem, "rdf:resource"))
              OntologyReader::push(individual_vector.object_relations_, Pair_t<std::string, std::string>(property, toString(sub_elem), probability), "$", "^");
            else if(testAttribute(sub_elem, "rdf:datatype"))
            {
              const char* value = sub_elem->GetText();
              if(value != nullptr)
              {
                LiteralNode data(toString(sub_elem, "rdf:datatype"), std::string(value));
                OntologyReader::push(individual_vector.data_relations_, Pair_t<std::string, std::string>(property, data.toString(), probability), "$", "^");
              }
            }
          }
        }
      }
    }
    individual_graph_->add(node_name, individual_vector);
    nb_loaded_elem_++;
  }
}

void OntologyOwlReader::readDescription(TiXmlElement* elem)
{
  std::vector<std::string> disjoints;
  bool is_all_disjoint_classes = false;
  bool is_all_disjoint_properties = false;

  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    std::string sub_elem_name = sub_elem->Value();
    const char* sub_attr;
    if(sub_elem_name == "rdf:type")
    {
      sub_attr = sub_elem->Attribute("rdf:resource");
      if(sub_attr != nullptr)
      {
        if(getName(std::string(sub_attr)) == "AllDisjointClasses")
          is_all_disjoint_classes = true;
        else if(getName(std::string(sub_attr)) == "AllDisjointProperties")
          is_all_disjoint_properties = true;
      }
    }
    else if(sub_elem_name == "owl:members")
    {
      sub_attr = sub_elem->Attribute("rdf:parseType");
      if(sub_attr != nullptr)
        if(std::string(sub_attr) == "Collection")
          readCollection(disjoints, sub_elem, "-");
    }
  }

  if(is_all_disjoint_classes)
    class_graph_->add(disjoints);
  else if(is_all_disjoint_properties)
    object_property_graph_->add(disjoints);
  disjoints.clear();
}

void OntologyOwlReader::readIndividualDescription(TiXmlElement* elem)
{
  std::string elem_name = elem->Value();
  if(elem_name == "rdf:Description")
  {
    std::vector<std::string> distincts;
    bool is_distinct_all = false;
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      std::string sub_elem_name = sub_elem->Value();
      const char* sub_attr;
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
      individual_graph_->add(distincts);
    distincts.clear();
  } // end if(elem_name == "rdf:Description")
}

void OntologyOwlReader::readObjectProperty(TiXmlElement* elem)
{
  std::string node_name = "";
  ObjectPropertyVectors_t property_vector;
  property_vector.annotation_usage_ = false;
  const char* attr = elem->Attribute("rdf:about");
  if(attr != nullptr)
  {
    node_name = getName(std::string(attr));
    if(display_)
      std::cout << "│   ├──" << node_name << std::endl;
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      std::string sub_elem_name = sub_elem->Value();
      float probability = getProbability(sub_elem);

      if(sub_elem_name == "rdfs:subPropertyOf")
        push(property_vector.mothers_, sub_elem, probability, "+");
      else if(sub_elem_name == "owl:disjointWith")
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
      else if(sub_elem_name == "owl:propertyChainAxiom")
      {
        std::vector<std::string> tmp;
        readCollection(tmp, sub_elem, ".", 2);
        property_vector.chains_.push_back(tmp);
      }
    }
  }

  object_property_graph_->add(node_name, property_vector);
  nb_loaded_elem_++;
}

void OntologyOwlReader::readDataProperty(TiXmlElement* elem)
{
  std::string node_name = "";
  DataPropertyVectors_t property_vector;
  property_vector.annotation_usage_ = false;
  const char* attr = elem->Attribute("rdf:about");
  if(attr != nullptr)
  {
    node_name = getName(std::string(attr));
    if(display_)
      std::cout << "│   ├──" << node_name << std::endl;
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      std::string sub_elem_name = sub_elem->Value();
      float probability = getProbability(sub_elem);

      if(sub_elem_name == "rdfs:subPropertyOf")
        push(property_vector.mothers_, sub_elem, probability, "+");
      else if(sub_elem_name == "owl:disjointWith")
        push(property_vector.disjoints_, sub_elem, probability, "-");
      else if(sub_elem_name == "rdfs:domain")
        push(property_vector.domains_, sub_elem, probability, ">");
      else if(sub_elem_name == "rdfs:range")
        push(property_vector.ranges_, sub_elem, "<");
      else if(sub_elem_name == "rdfs:label")
        pushLang(property_vector.dictionary_, sub_elem);
      else if(sub_elem_name == "onto:label")
        pushLang(property_vector.muted_dictionary_, sub_elem);
    }
  }

  data_property_graph_->add(node_name, property_vector);
  nb_loaded_elem_++;
}

void OntologyOwlReader::readAnnotationProperty(TiXmlElement* elem)
{
  std::string node_name = "";
  DataPropertyVectors_t property_vector; // we use a DataPropertyVectors_t that is sufficient to represent an annotation property
  property_vector.annotation_usage_ = true;
  std::vector<Single_t<std::string>> ranges_;
  const char* attr = elem->Attribute("rdf:about");
  if(attr != nullptr)
  {
    node_name = getName(std::string(attr));
    if(display_)
      std::cout << "│   ├──" << node_name << std::endl;
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      std::string sub_elem_name = sub_elem->Value();
      float probability = getProbability(sub_elem);

      if(sub_elem_name == "rdfs:subPropertyOf")
        push(property_vector.mothers_, sub_elem, probability, "+");
      else if(sub_elem_name == "owl:disjointWith")
        push(property_vector.disjoints_, sub_elem, probability, "-");
      else if(sub_elem_name == "rdfs:domain")
        push(property_vector.domains_, sub_elem, probability, ">");
      else if(sub_elem_name == "rdfs:range")
      {
        push(property_vector.ranges_, sub_elem, "<");
        push(ranges_, sub_elem, probability);
      }
      else if(sub_elem_name == "rdfs:label")
        pushLang(property_vector.dictionary_, sub_elem);
      else if(sub_elem_name == "onto:label")
        pushLang(property_vector.muted_dictionary_, sub_elem);
    }
  }

  // data_property_graph_ will return false if no data property is found with this name
  if(data_property_graph_->addAnnotation(node_name, property_vector) == false)
  {
    ObjectPropertyVectors_t object_property_vector;
    object_property_vector.mothers_ = property_vector.mothers_;
    object_property_vector.disjoints_ = property_vector.disjoints_;
    object_property_vector.domains_ = property_vector.domains_;
    object_property_vector.ranges_ = ranges_;
    object_property_vector.dictionary_ = property_vector.dictionary_;
    object_property_vector.muted_dictionary_ = property_vector.muted_dictionary_;
    object_property_vector.annotation_usage_ = true;

    object_property_graph_->add(node_name, object_property_vector);
    // if no data property is found, the annotation will be setted as an object property by default
  }

  nb_loaded_elem_++;
}

void OntologyOwlReader::readCollection(std::vector<std::string>& vect, TiXmlElement* elem, const std::string& symbol, size_t level)
{
  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    std::string sub_elem_name = sub_elem->Value();
    if(sub_elem_name == "rdf:Description")
    {
      const char* sub_attr = sub_elem->Attribute("rdf:about");
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

std::string OntologyOwlReader::readSomeValuesFrom(TiXmlElement* elem)
{
  std::string value = getAttribute(elem, "rdf:resource");
  if(value == "")
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
    {
      std::string restriction_name = sub_elem->Value();
      if(restriction_name == "rdfs:Datatype" && display_)
        std::cout << restriction_name << std::endl;
    }
  return value;
}

void OntologyOwlReader::push(Properties_t& properties, TiXmlElement* sub_elem, const std::string& symbole, const std::string& attribute)
{
  const char* sub_attr;
  sub_attr = sub_elem->Attribute(attribute.c_str());
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

    if(property != "" && display_)
      std::cout << "│   │   ├── " << symbole << property << std::endl;
  }
}

void OntologyOwlReader::pushLang(std::map<std::string, std::vector<std::string>>& dictionary, TiXmlElement* sub_elem)
{
  const char* sub_attr;
  sub_attr = sub_elem->Attribute("xml:lang");
  if(sub_attr != nullptr)
  {
    std::string lang = std::string(sub_attr);

    const char* value;
    value = sub_elem->GetText();
    if(value != nullptr)
    {
      dictionary[lang].push_back(std::string(value));

      if((lang != "") && (std::string(value) != "") && display_)
        std::cout << "│   │   ├── " << "@" << lang << " : " << dictionary[lang][dictionary[lang].size() - 1] << std::endl;
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
          pose = pose-2;
          txt.erase(pose, i-pose+1);
          return;
        }
      }
    }
  }
}

} // namespace ontologenius
