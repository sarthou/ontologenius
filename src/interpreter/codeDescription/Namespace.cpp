#include "ontoloGenius/interpreter/codeDescription/Namespace.h"

Namespace::Namespace(std::string name)
{
  name_ = name;
}

bool Namespace::isThisNamespace(std::string ns)
{
  if(ns == name_)
    return true;
  else
    return false;
}
