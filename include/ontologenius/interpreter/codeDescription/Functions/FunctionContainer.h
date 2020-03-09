#ifndef ONTOLOGENIUS_FUNCTIONCONTAINER_H
#define ONTOLOGENIUS_FUNCTIONCONTAINER_H

#include <vector>

#include "ontologenius/interpreter/codeDescription/Functions/FunctionDescriptor.h"

namespace ontologenius {

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

} // namespace ontologenius

#endif // ONTOLOGENIUS_FUNCTIONCONTAINER_H
