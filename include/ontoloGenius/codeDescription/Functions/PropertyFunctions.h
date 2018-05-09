#ifndef PROPERTYFUNCTIONS_H
#define PROPERTYFUNCTIONS_H

#include <string>

#include "ontoloGenius/codeDescription/Functions/FunctionContainer.h"

class PropertyFunctions: public FunctionContainer
{
public:
  PropertyFunctions() {}
  ~PropertyFunctions() {}

  bool functionExist(std::string name);
  FunctionDescriptor* findFunction(std::string name);

private:
};

#endif
