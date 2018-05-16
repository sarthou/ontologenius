#include "ontoloGenius/codeDescription/Functions/PropertyFunctions.h"

bool PropertyFunctions::functionExist(std::string name)
{
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
