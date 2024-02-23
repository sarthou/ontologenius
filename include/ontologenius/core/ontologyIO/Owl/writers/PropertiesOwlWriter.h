#ifndef ONTOLOGENIUS_PROPERTIESOWLWRITER_H
#define ONTOLOGENIUS_PROPERTIESOWLWRITER_H

#include <string>

#include "ontologenius/core/ontologyIO/Owl/writers/NodeOwlWriter.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

template <typename T>
class PropertiesOwlWriter : public NodeOwlWriter
{
public:
  PropertiesOwlWriter() {};
  ~PropertiesOwlWriter() {};

protected:

  void writeDisjointWith(Branch_t<T>* branch);
  void writeProperties(PropertyBranch_t<T>* branch);
};

template <typename T>
void PropertiesOwlWriter<T>::writeDisjointWith(Branch_t<T>* branch)
{
  for(auto& disjoint : branch->disjoints_)
    if(disjoint.infered == false)
    {
      std::string tmp = "        <owl:disjointWith" +
                        getProba(disjoint) +
                        " rdf:resource=\"" + ns_ + "#" +
                        disjoint.elem->value()
                        + "\"/>\n\r";
      writeString(tmp);
    }
}

template <typename T>
void PropertiesOwlWriter<T>::writeProperties(PropertyBranch_t<T>* branch)
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

#endif // ONTOLOGENIUS_PROPERTIESOWLWRITER_H
