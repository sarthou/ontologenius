#include "ontologenius/interpreter/codeDescription/Functions/FunctionContainer.h"

namespace ontologenius {

bool FunctionContainer::functionExist(std::string name)
{
  for(size_t i = 0; i < functions_.size(); i++)
  {
    if(functions_[i].getName() == name)
      return true;
  }
  return false;
}

FunctionDescriptor* FunctionContainer::findFunction(std::string name)
{
  for(size_t i = 0; i < functions_.size(); i++)
  {
    if(functions_[i].getName() == name)
      return &functions_[i];
  }
  return nullptr;
}

} // namespace ontologenius
