#ifndef ONTOLOGENIUS_ONTOLOGYOWLREADER_H
#define ONTOLOGENIUS_ONTOLOGYOWLREADER_H

#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
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

    int read(tinyxml2::XMLElement* rdf, const std::string& name);
    int readIndividual(tinyxml2::XMLElement* rdf, const std::string& name);
    void readClass(tinyxml2::XMLElement* elem);
    void readIndividual(tinyxml2::XMLElement* elem);
    void readDescription(tinyxml2::XMLElement* elem);
    void readIndividualDescription(tinyxml2::XMLElement* elem);
    void readObjectProperty(tinyxml2::XMLElement* elem);
    void readDataProperty(tinyxml2::XMLElement* elem);
    void readAnnotationProperty(tinyxml2::XMLElement* elem);
    void readCollection(std::vector<std::string>& vect, tinyxml2::XMLElement* elem, const std::string& symbol, size_t level = 1);
    std::string readSomeValuesFrom(tinyxml2::XMLElement* elem);
    void removeDocType(std::string& txt);
    void readDisjoint(tinyxml2::XMLElement* elem, bool is_class);

    /*************************
     * Anonymous Class Reader *
     *************************/

    std::unordered_map<std::string, std::string> card_map_;

    void readEquivalentClass(AnonymousClassVectors_t& ano, tinyxml2::XMLElement* elem, const std::string& class_name);
    ExpressionMember_t* readAnonymousRestriction(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readAnonymousClassExpression(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readAnonymousDatatypeExpression(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readAnonymousIntersection(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readAnonymousUnion(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readAnonymousOneOf(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readAnonymousComplement(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readAnonymousComplexDescription(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readAnonymousResource(tinyxml2::XMLElement* elem, const std::string& attribute_name = "rdf:resource");

    void addAnonymousChildMember(ExpressionMember_t* parent, ExpressionMember_t* child, tinyxml2::XMLElement* used_elem);

    bool readAnonymousCardinalityRange(tinyxml2::XMLElement* elem, ExpressionMember_t* exp);
    void readAnonymousCardinalityValue(tinyxml2::XMLElement* elem, ExpressionMember_t* exp);

    /**********************
     *   SWRL Rule Reader  *
     **********************/
    void readRuleDescription(Rule_t& rule, tinyxml2::XMLElement* elem);

    ExpressionMember_t* readRuleRestriction(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readRuleClassExpression(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readRuleDatatypeExpression(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readRuleIntersection(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readRuleUnion(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readRuleOneOf(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readRuleComplement(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readRuleComplexDescription(tinyxml2::XMLElement* elem);
    ExpressionMember_t* readRuleResource(tinyxml2::XMLElement* elem, const std::string& attribute_name = "rdf:resource");

    void addRuleChildMember(ExpressionMember_t* parent, ExpressionMember_t* child, tinyxml2::XMLElement* used_elem);

    bool readRuleCardinalityRange(tinyxml2::XMLElement* elem, ExpressionMember_t* exp);
    void readRuleCardinalityValue(tinyxml2::XMLElement* elem, ExpressionMember_t* exp);

    void readRuleCollection(tinyxml2::XMLElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>>& exp_vect);
    void readSwrlRule(tinyxml2::XMLElement* elem);
    std::pair<ExpressionMember_t*, std::vector<Variable_t>> readRuleAtom(tinyxml2::XMLElement* elem, const std::string& type_atom);
    std::pair<ExpressionMember_t*, std::vector<Variable_t>> readRuleClassAtom(tinyxml2::XMLElement* elem);
    std::pair<ExpressionMember_t*, std::vector<Variable_t>> readRuleObjectPropertyAtom(tinyxml2::XMLElement* elem);
    std::pair<ExpressionMember_t*, std::vector<Variable_t>> readRuleDataPropertyAtom(tinyxml2::XMLElement* elem);
    std::pair<ExpressionMember_t*, std::vector<Variable_t>> readRuleBuiltinAtom(tinyxml2::XMLElement* elem);

    void readRestAtom(tinyxml2::XMLElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>>& exp_vect);
    void readFirstAtom(tinyxml2::XMLElement* elem, std::vector<std::pair<ExpressionMember_t*, std::vector<Variable_t>>>& exp_vect);
    Variable_t getRuleArgument(tinyxml2::XMLElement* elem);
    std::vector<Variable_t> readRuleBuiltinArguments(tinyxml2::XMLElement* elem);
    void readSimpleBuiltinArguments(tinyxml2::XMLElement* elem, std::vector<Variable_t>& variables);
    void readComplexBuiltinArguments(tinyxml2::XMLElement* elem, std::vector<Variable_t>& variables);

    /**********************
     *        inline       *
     **********************/

    inline void push(std::vector<std::string>& vect, tinyxml2::XMLElement* sub_elem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    inline void push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole = "");
    inline void push(std::vector<SingleElement<std::string>>& vect, tinyxml2::XMLElement* sub_elem, float probability, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    inline void push(std::vector<bool>& vect, bool elem, const std::string& symbole = "");
    void push(Properties_t& properties, tinyxml2::XMLElement* sub_elem, const std::string& symbole = "", const std::string& attribute = "rdf:resource");
    void pushLang(std::map<std::string, std::vector<std::string>>& dictionary, tinyxml2::XMLElement* sub_elem);
    inline std::string getName(const std::string& uri);
    inline float getProbability(tinyxml2::XMLElement* elem);
    inline std::string getAttribute(tinyxml2::XMLElement* elem, const std::string& attribute);
    inline bool testAttribute(tinyxml2::XMLElement* sub_elem, const std::string& attribute);
    inline int getNbChildren(tinyxml2::XMLElement* elem);

    std::string toString(tinyxml2::XMLElement* sub_elem, const std::string& attribute = "rdf:resource")
    {
      const char* sub_attr = sub_elem->Attribute(attribute.c_str());
      if(sub_attr != nullptr)
        return getName(std::string(sub_attr));
      return "";
    }
  };

  void OntologyOwlReader::push(std::vector<std::string>& vect, tinyxml2::XMLElement* sub_elem, const std::string& symbole, const std::string& attribute)
  {
    std::string data = getAttribute(sub_elem, attribute);
    if(data.empty() == false)
    {
      vect.push_back(data);
      if(symbole.empty() == false && display_)
        std::cout << "│   │   ├── " << symbole << " " << data << std::endl;
    }
  }

  void OntologyOwlReader::push(std::vector<SingleElement<std::string>>& vect, tinyxml2::XMLElement* sub_elem, float probability, const std::string& symbole, const std::string& attribute)
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

  float OntologyOwlReader::getProbability(tinyxml2::XMLElement* elem)
  {
    float proba = 1.0;

    const char* sub_attr = elem->Attribute("onto:probability");
    if(sub_attr != nullptr)
      proba = std::stof(std::string(sub_attr));

    return proba;
  }

  inline std::string OntologyOwlReader::getAttribute(tinyxml2::XMLElement* elem, const std::string& attribute)
  {
    const char* sub_attr = elem->Attribute(attribute.c_str());
    if(sub_attr != nullptr)
      return getName(std::string(sub_attr));
    else
      return "";
  }

  bool OntologyOwlReader::testAttribute(tinyxml2::XMLElement* sub_elem, const std::string& attribute)
  {
    const char* sub_attr = sub_elem->Attribute(attribute.c_str());
    if(sub_attr != nullptr)
      return true;
    else
      return false;
  }

  int OntologyOwlReader::getNbChildren(tinyxml2::XMLElement* elem)
  {
    int cpt = 0;
    for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      cpt++;
    return cpt;
  }
} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYOWLREADER_H
