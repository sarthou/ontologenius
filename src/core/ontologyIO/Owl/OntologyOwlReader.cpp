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
      std::cout << "/ inverse      | * type    | = equivalence" << std::endl;
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
  AnonymousClassVectors_t ano_vector;
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
        readEquivalentClass(ano_vector, sub_elem, attr);
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

  if(ano_vector.equivalence_trees.size() != 0)
  {
    anonymous_graph_->add(node_name, ano_vector);
    for(auto& str_elem : ano_vector.str_equivalences)
      push(object_vector.equivalences_, str_elem , "=");
  }
 
  class_graph_->add(node_name, object_vector);
  nb_loaded_elem_++;
}

void OntologyOwlReader::readEquivalentClass(AnonymousClassVectors_t& ano, TiXmlElement* elem, const std::string& class_name)
{
  ano.class_equiv = class_name;
  ExpressionMember_t* exp;

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
    std::string sub_elem_name = sub_elem->Value();

    // Restriction equivalence : Camera Eq to hasComponent some Component
    if(sub_elem_name == "owl:Restriction")
      exp = readRestriction(sub_elem);
    // Logical expression equivalence : Camera Eq to (hasComponent some Component and has_node only rational)
    else if(sub_elem_name == "owl:Class")
      exp = readClassExpression(sub_elem);
  }

  ano.equivalence_trees.push_back(exp);
  ano.str_equivalences.push_back(exp->toString());
}

ExpressionMember_t* OntologyOwlReader::readRestriction(TiXmlElement* elem)
{
  ExpressionMember_t* exp = new ExpressionMember_t();

  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    std::string sub_elem_name = sub_elem->Value();

    if(sub_elem_name == "owl:onProperty")
      exp->rest.property = getName(sub_elem->Attribute("rdf:resource"));
    else if((sub_elem_name == "owl:maxQualifiedCardinality") ||
            (sub_elem_name == "owl:minQualifiedCardinality") ||
            (sub_elem_name == "owl:qualifiedCardinality"))
      readCardinalityValue(sub_elem, exp);
    else if((sub_elem_name == "owl:allValuesFrom") ||
            (sub_elem_name == "owl:hasValue") ||
            (sub_elem_name == "owl:someValuesFrom"))
    {
      if(readCardinalityRange(sub_elem, exp) == false)
      {
        exp->is_complex = true;
        addChildMember(exp, readComplexDescription(sub_elem->FirstChildElement()), sub_elem->FirstChildElement());
      }
    }
    else if(sub_elem_name == "owl:onClass" || sub_elem_name == "owl:onDataRange")
    {
      const char* resource = sub_elem->Attribute("rdf:resource");
      // Simple restriction range : Camera Eq to  hasComponent some Lidar
      // <owl:onClass rdf:resource="test_bastien#Lidar"/>
      if(resource != nullptr)
      {
        std::string attr_class = sub_elem->Attribute("rdf:resource");
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
        addChildMember(exp, readComplexDescription(sub_elem->FirstChildElement()), sub_elem->FirstChildElement());
      }
    }
  }

  return exp;
}

ExpressionMember_t* OntologyOwlReader::readClassExpression(TiXmlElement* elem)
{
  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    std::string sub_elem_name = sub_elem->Value();
    if(sub_elem_name == "owl:unionOf")
      return readUnion(sub_elem);
    else if(sub_elem_name == "owl:intersectionOf")
      return readIntersection(sub_elem);
    else if(sub_elem_name == "owl:complementOf")
      return readComplement(sub_elem);
    else if(sub_elem_name == "owl:oneOf")
      return readOneOf(sub_elem);
  }

  return nullptr;
}

ExpressionMember_t* OntologyOwlReader::readDatatypeExpression(TiXmlElement* elem)
{
  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    ExpressionMember_t* exp = nullptr;
    std::string sub_elem_name = sub_elem->Value();
    if(sub_elem_name == "owl:unionOf")
      exp = readUnion(sub_elem);
    else if(sub_elem_name == "owl:intersectionOf")
      exp = readIntersection(sub_elem);
    else if(sub_elem_name == "owl:datatypeComplementOf")
      exp = readComplement(sub_elem);

    if(exp != nullptr)
    {
      exp->is_data_property = true;
      return exp;
    }
  }

  return nullptr;
}

ExpressionMember_t* OntologyOwlReader::readIntersection(TiXmlElement* elem)
{
  ExpressionMember_t* exp = new ExpressionMember_t();
  exp->logical_type_ = logical_and;

  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    ExpressionMember_t* child_exp = readComplexDescription(sub_elem);
    addChildMember(exp, child_exp, sub_elem);
  }
  
  return exp;
}

ExpressionMember_t* OntologyOwlReader::readUnion(TiXmlElement* elem)
{
  ExpressionMember_t* exp = new ExpressionMember_t();
  exp->logical_type_ = logical_or;

  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    ExpressionMember_t* child_exp = readComplexDescription(sub_elem);
    addChildMember(exp, child_exp, sub_elem);
  }

  return exp;
}

ExpressionMember_t* OntologyOwlReader::readOneOf(TiXmlElement* elem)
{
  ExpressionMember_t* exp = new ExpressionMember_t();
  exp->oneof = true;

  for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
  {
    ExpressionMember_t* child_exp = readResource(sub_elem, "rdf:about");
    addChildMember(exp, child_exp, sub_elem);
  }

  return exp;
}

ExpressionMember_t* OntologyOwlReader::readComplement(TiXmlElement* elem)
{
  ExpressionMember_t* exp = new ExpressionMember_t();
  exp->logical_type_ = logical_not;
  ExpressionMember_t* child_exp = nullptr;

  const char* resource = elem->Attribute("rdf:resource");

  if(resource != nullptr)
    child_exp = readResource(elem);
  else
    child_exp = readComplexDescription(elem->FirstChildElement());

  addChildMember(exp, child_exp, elem->FirstChildElement());

  return exp;
}

ExpressionMember_t* OntologyOwlReader::readComplexDescription(TiXmlElement* elem)
{
  if(elem == nullptr)
    return nullptr;

  std::string elem_name = elem->Value();
  if(elem_name == "owl:Class")
    return readClassExpression(elem);
  else if(elem_name == "rdfs:Datatype")
    return readDatatypeExpression(elem);
  else if(elem_name == "owl:Restriction")
    return readRestriction(elem);
  else if(elem_name == "rdf:Description")
    return readResource(elem, "rdf:about");
  else
    return nullptr;
}

ExpressionMember_t* OntologyOwlReader::readResource(TiXmlElement* elem, const std::string& attribute_name)
{
  ExpressionMember_t* exp = nullptr;

  const char* resource = elem->Attribute(attribute_name.c_str());
  if(resource != nullptr)
  {
    exp = new ExpressionMember_t();

    std::string attr_class = elem->Attribute(attribute_name.c_str());
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

void OntologyOwlReader::addChildMember(ExpressionMember_t* parent, ExpressionMember_t* child, TiXmlElement* used_elem)
{
  if(child != nullptr)
  {
    parent->child_members.push_back(child);
    child->mother = parent;
    if((used_elem != nullptr) && (std::string(used_elem->Value()) != "owl:Restriction"))
      parent->is_data_property = child->is_data_property;
  }
}

void OntologyOwlReader::readCardinalityValue(TiXmlElement* elem, ExpressionMember_t* exp)
{
  std::string sub_elem_name = elem->Value();

  exp->rest.card.cardinality_type = card_map_[sub_elem_name];

  if(elem->GetText() != nullptr)
    exp->rest.card.cardinality_number = elem->GetText();
}

bool OntologyOwlReader::readCardinalityRange(TiXmlElement* elem, ExpressionMember_t* exp)
{
  std::string sub_elem_name = elem->Value();

  const char* resource = elem->Attribute("rdf:resource");
  exp->rest.card.cardinality_type = card_map_[sub_elem_name];

  if(resource != nullptr)
  {
    std::string s = resource;
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
