#ifndef ONTOLOGENIUS_ONTOFUNCTIONS_H
#define ONTOLOGENIUS_ONTOFUNCTIONS_H

#include <string>
#include <map>
#include <unordered_set>
#include <stdint.h>

#include "ontoloGenius/interpreter/codeDescription/Namespace.h"
#include "ontoloGenius/interpreter/codeDescription/Functions/FunctionContainer.h"

namespace ontologenius {

class OntoFunctions : public Namespace, public FunctionContainer
{
public:
  OntoFunctions();
  ~OntoFunctions() {};

private:
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOFUNCTIONS_H
