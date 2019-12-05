#ifndef ONTOLOGENIUS_PROPERTYFUNCTIONS_H
#define ONTOLOGENIUS_PROPERTYFUNCTIONS_H

#include <string>

#include "ontologenius/interpreter/codeDescription/Functions/FunctionContainer.h"

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

#endif // ONTOLOGENIUS_PROPERTYFUNCTIONS_H
