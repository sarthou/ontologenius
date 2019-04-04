#ifndef PROPERTYFUNCTIONS_H
#define PROPERTYFUNCTIONS_H

#include <string>

#include "ontoloGenius/interpreter/codeDescription/Functions/FunctionContainer.h"

namespace ontologenius {

class PropertyFunctions: public FunctionContainer
{
public:
  PropertyFunctions() {}
  ~PropertyFunctions() {}

  bool functionExist(std::string name);
  FunctionDescriptor* findFunction(std::string name);

private:
};

} // namespace ontologenius

#endif
