#ifndef ONTOLOGENIUS_PROPERTIESWRITER_H
#define ONTOLOGENIUS_PROPERTIESWRITER_H

#include <string>

#include "ontoloGenius/core/ontologyIO/writers/NodeWriter.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

template <typename T>
class PropertiesWriter : public NodeWriter
{
public:
  PropertiesWriter() {};
  ~PropertiesWriter() {};

protected:

  void writeDisjointWith(PropertyBranchData_t<T>* branch);
  void writeProperties(PropertyBranchData_t<T>* branch);
};

template <typename T>
void PropertiesWriter<T>::writeDisjointWith(PropertyBranchData_t<T>* branch)
{
  for(size_t i = 0; i < branch->disjoints_.size(); i++)
  {
    std::string tmp = "        <owl:disjointWith rdf:resource=\"ontologenius#" +
                      branch->disjoints_[i]->value()
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

template <typename T>
void PropertiesWriter<T>::writeProperties(PropertyBranchData_t<T>* branch)
{
  std::string tmp;
  if(branch->properties_.functional_property_ == true)
    tmp += "        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#FunctionalProperty\"/>\n\r";
  if(branch->properties_.inverse_functional_property_ == true)
    tmp += "        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#InverseFunctionalProperty\"/>\n\r";
  if(branch->properties_.transitive_property_ == true)
    tmp += "        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#TransitiveProperty\"/>\n\r";
  if(branch->properties_.symetric_property_ == true)
    tmp += "        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#SymmetricProperty\"/>\n\r";
  if(branch->properties_.antisymetric_property_ == true)
    tmp += "        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AsymmetricProperty\"/>\n\r";
  if(branch->properties_.reflexive_property_ == true)
    tmp += "        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#ReflexiveProperty\"/>\n\r";
  if(branch->properties_.irreflexive_property_ == true)
    tmp += "        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#IrreflexiveProperty\"/>\n\r";

  if(tmp != "")
    writeString(tmp);
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_PROPERTIESWRITER_H
