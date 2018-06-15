#ifndef PROPERTIESWRITER_H
#define PROPERTIESWRITER_H

#include "ontoloGenius/core/ontoGraphs/writers/NodeWriter.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/PropertyBranch.h"

#include <string>

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
                      branch->disjoints_[i]->value_
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

template <typename T>
void PropertiesWriter<T>::writeProperties(PropertyBranchData_t<T>* branch)
{

}

#endif
