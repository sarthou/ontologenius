#ifndef ONTOFUNCTIONS_H
#define ONTOFUNCTIONS_H

#include <string>
#include <map>
#include <set>

#include <stdint.h>

#include "ontoloGenius/codeDescription/Namespace.h"
#include "ontoloGenius/codeDescription/Functions/FunctionContainer.h"

class OntoFunctions : public Namespace, public FunctionContainer
{
public:
  OntoFunctions();
  ~OntoFunctions() {};

private:
};

#endif
