#include "ontoloGenius/codeDescription/Functions/OntoFunctions.h"

OntoFunctions::OntoFunctions() : Namespace("ont")
{
  FunctionDescriptor print = FunctionDescriptor("print", type_void, std::vector<type_t>(1, type_string));
  functions_.push_back(print);

  FunctionDescriptor null = FunctionDescriptor("null", type_bool, std::vector<type_t>(1, type_void));
  functions_.push_back(null);
}
