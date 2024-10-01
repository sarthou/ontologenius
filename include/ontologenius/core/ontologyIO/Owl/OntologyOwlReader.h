#ifndef ONTOLOGENIUS_ONTOLOGYOWLREADER_H
#define ONTOLOGENIUS_ONTOLOGYOWLREADER_H

#include <iostream>
#include <map>
#include <string>
#include <tinyxml.h>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"
#include "ontologenius/core/ontologyIO/OntologyReader.h"

namespace ontologenius {

  class Ontology;

  class OntologyOwlReader : public OntologyReader
  {
  public:
    OntologyOwlReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph, RuleGraph* rule_graph);
    explicit OntologyOwlReader(Ontology& onto);
    ~OntologyOwlReader() = default;

    int readFromUri(std::string content, const std::string& uri, bool individual = false);
    int readFromFile(const std::string& file_name, bool individual = false);

    std::vector<std::string> getImportsFromRaw(std::string content);
    std::vector<std::string> getImportsFromFile(const std::string& file_name);

    void setDisplay(bool display) { display_ = display; }
    void displayIndividualRules();
    bool empty() const { return (nb_loaded_elem_ == 0); }

  private:
    /**********************
     *      Owl Reader     *
     **********************/

    int read(TiXmlElement* rdf, const std::string& name);
    int readIndividual(TiXmlElement* rdf, const std::string& name);
    void readClass(TiXmlElement* elem);
    void readIndividual(TiXmlElement* elem);
    void readDescription(TiXmlElement* elem);
    void readIndividualDescription(TiXmlElement* elem);
    void readObjectProperty(TiXmlElement* elem);
    void readDataProperty(TiXmlElement* elem);
    void readAnnotationProperty(TiXmlElement* elem);
    void readCollection(std::vector<std::string>& vect, TiXmlElement* elem, const std::string& symbol, size_t level = 1);
    std::string readSomeValuesFrom(TiXmlElement* elem);
    void removeDocType(std::string& txt);
    void readDisjoint(TiXmlElement* elem, bool is_class);

    /*************************
     * Anonymous Class Reader *
     *************************/

    std::unordered_map<std::string, std::string> card_map_;

    void readEquivalentClass(AnonymousClassVectors_t& ano, TiXmlElement* elem, const std::string& class_name);
    ExpressionMember_t* readAnonymousRestriction(TiXmlElement* elem);
    ExpressionMember_t* readAnonymousClassExpression(TiXmlElement* elem);
    ExpressionMember_t* readAnonymousDatatypeExpression(TiXmlElement* elem);
    ExpressionMember_t* readAnonymousIntersection(TiXmlElement* elem);
    ExpressionMember_t* readAnonymousUnion(TiXmlElement* elem);
    ExpressionMember_t* readAnonymousOneOf(TiXmlElement* elem);
    ExpressionMember_t* readAnonymousComplement(TiXmlElement* elem);
    ExpressionMember_t* readAnonymousComplexDescription(TiXmlElement* elem);
    ExpressionMember_t* readAnonymousResource(TiXmlElement* elem, const std::string& attribute_name = "rdf:resource");

    void addAnonymousChildMember(ExpressionMember_t* parent, ExpressionMember_t* child, TiXmlElement* used_elem);

    bool readAnonymousCardinalityRange(TiXmlElement* elem, ExpressionMember_t* exp);
    void readAnonymousCardinalityValue(TiXmlElement* elem, ExpressionMember_t* exp);

    /**********************
     *   SWRL Rule Reader  *
     **********************/
    void readRuleDescription(Rule_t& rule, TiXmlElement* elem);

    ExpressionMember_t* readRuleRestriction(TiXmlElement* elem);
    ExpressionMember_t* readRuleClassExpression(TiXmlElement* elem);
    ExpressionMember_t* readRuleDatatypeExpression(TiXmlElement* elem);
    ExpressionMember_t* readRuleIntersection(TiXmlElement* elem);
    ExpressionMember_t* readRuleUnion(TiXmlElement* elem);
    ExpressionMember_t* readRuleOneOf(TiXmlElement* elem);
    ExpressionMember_t* readRuleComplement(TiXmlElement* elem);
    ExpressionMember_t* readRuleComplexDescription(TiXmlElement* elem);
    ExpressionMember_t* readRuleResource(TiXmlElement* elem, const std::string& attribute_name = "rdf:resource");

    void addRuleChildMember(ExpressionMember_t* parent, ExpressionMember_t* child, TiXmlElement* used_elem);

    bool readRuleCardinalityRange(TiXmlElement* elem, ExpressionMember_t* exp);
    void readRuleCardinalityValue(TiXmlElement* elem, ExpressionMember_t* exp);

    void readRuleCollection(TiXmlElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<std::string>>>& exp_vect);
    void readSwrlVariable(TiXmlElement* elem);
    void readSwrlRule(TiXmlElement* elem);
    std::pair<ExpressionMember_t*, std::vector<std::string>> readRuleAtom(TiXmlElement* elem, const std::string& type_atom);
    std::pair<ExpressionMember_t*, std::vector<std::string>> readRuleClassAtom(TiXmlElement* elem);
    std::pair<ExpressionMember_t*, std::vector<std::string>> readRuleObjectPropertyAtom(TiXmlElement* elem);
    std::pair<ExpressionMember_t*, std::vector<std::string>> readRuleDataPropertyAtom(TiXmlElement* elem);
    std::pair<ExpressionMember_t*, std::vector<std::string>> readRuleBuiltinAtom(TiXmlElement* elem);

    void readRestAtom(TiXmlElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<std::string>>>& exp_vect);
    void readFirstAtom(TiXmlElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<std::string>>>& exp_vect);
    std::string getRuleArgument(TiXmlElement* elem);

    /**********************
     *        inline       *
     **********************/

    inline void push(std::vector<std::string>& vect, TiXmlElement* sub_elem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    inline void push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole = "");
    inline void push(std::vector<SingleElement<std::string>>& vect, TiXmlElement* sub_elem, float probability, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    inline void push(std::vector<bool>& vect, bool elem, const std::string& symbole = "");
    void push(Properties_t& properties, TiXmlElement* sub_elem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    void pushLang(std::map<std::string, std::vector<std::string>>& dictionary, TiXmlElement* sub_elem);
    inline std::string getName(const std::string& uri);
    inline float getProbability(TiXmlElement* elem);
    inline std::string getAttribute(TiXmlElement* elem, const std::string& attribute);
    inline bool testAttribute(TiXmlElement* sub_elem, const std::string& attribute);
    inline int getNbChildren(TiXmlElement* elem);

    std::string toString(TiXmlElement* sub_elem, const std::string& attribute = "rdf:resource")
    {
      const char* sub_attr = sub_elem->Attribute(attribute.c_str());
      if(sub_attr != nullptr)
        return getName(std::string(sub_attr));
      return "";
    }
  };

  void OntologyOwlReader::push(std::vector<std::string>& vect, TiXmlElement* sub_elem, const std::string& symbole, const std::string& attribute)
  {
    std::string data = getAttribute(sub_elem, attribute);
    if(data.empty() == false)
    {
      vect.push_back(data);
      if(symbole.empty() == false && display_)
        std::cout << "│   │   ├── " << symbole << " " << data << std::endl;
    }
  }

  void OntologyOwlReader::push(std::vector<SingleElement<std::string>>& vect, TiXmlElement* sub_elem, float probability, const std::string& symbole, const std::string& attribute)
  {
    std::string data = getAttribute(sub_elem, attribute);
    if(data.empty() == false)
    {
      vect.emplace_back(data, probability);
      if(symbole.empty() == false && display_)
        std::cout << "│   │   ├── " << symbole << " " << data << std::endl;
    }
  }

  void OntologyOwlReader::push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole)
  {
    const std::string& data = elem;
    if(data.empty() == false)
    {
      vect.push_back(data);
      if((symbole.empty() == false) && display_)
        std::cout << "│   │   ├── " << symbole << " " << data << std::endl;
    }
  }

  std::string OntologyOwlReader::getName(const std::string& uri)
  {
    size_t pos = uri.find('#');
    std::string result = uri.substr(pos + 1);
    return result;
  }

  float OntologyOwlReader::getProbability(TiXmlElement* elem)
  {
    float proba = 1.0;

    const char* sub_attr = elem->Attribute("onto:probability");
    if(sub_attr != nullptr)
      proba = std::stof(std::string(sub_attr));

    return proba;
  }

  inline std::string OntologyOwlReader::getAttribute(TiXmlElement* elem, const std::string& attribute)
  {
    const char* sub_attr = elem->Attribute(attribute.c_str());
    if(sub_attr != nullptr)
      return getName(std::string(sub_attr));
    else
      return "";
  }

  bool OntologyOwlReader::testAttribute(TiXmlElement* sub_elem, const std::string& attribute)
  {
    const char* sub_attr = sub_elem->Attribute(attribute.c_str());
    if(sub_attr != nullptr)
      return true;
    else
      return false;
  }

  int OntologyOwlReader::getNbChildren(TiXmlElement* elem)
  {
    int cpt = 0;
    for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      cpt++;
    return cpt;
  }
} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYOWLREADER_H
