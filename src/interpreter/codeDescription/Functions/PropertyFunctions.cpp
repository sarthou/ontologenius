#include "ontologenius/interpreter/codeDescription/Functions/PropertyFunctions.h"

namespace ontologenius {

bool PropertyFunctions::functionExist(std::string name)
{
  (void)name;
  return true;
}

FunctionDescriptor* PropertyFunctions::findFunction(std::string name)
{
  for(size_t i = 0; i < functions_.size(); i++)
  {
    if(functions_[i].getName() == name)
      return &functions_[i];
  }

  FunctionDescriptor tmp = FunctionDescriptor(name, type_word_set, std::vector<type_t>(1, type_void));
  functions_.push_back(tmp);
  return &functions_[functions_.size() - 1];
}

} // namespace ontologenius
