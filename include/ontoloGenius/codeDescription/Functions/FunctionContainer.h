#ifndef FUNCTIONCONTAINER_H
#define FUNCTIONCONTAINER_H

#include <vector>

#include "ontoloGenius/codeDescription/Functions/FunctionDescriptor.h"

class FunctionContainer
{
public:
  FunctionContainer() {}
  ~FunctionContainer() {}

  bool functionExist(std::string name);
  FunctionDescriptor* findFunction(std::string name);

protected:
  std::vector<FunctionDescriptor> functions_;
};

#endif
