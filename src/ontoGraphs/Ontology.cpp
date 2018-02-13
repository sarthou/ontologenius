#include "ontoloGenius/ontoGraphs/Ontology.h"

#include "ontoloGenius/ontoGraphs/Checkers/ClassChecker.h"
#include "ontoloGenius/ontoGraphs/Checkers/PropertyChecker.h"

int Ontology::close()
{
  classes_.close();
  properties_.close();

  ClassChecker classChecker(&classes_);
  PropertyChecker propertyChecker(&properties_);

  size_t err = classChecker.check();
  err = propertyChecker.check();

  if(err)
    return -1;
  else
    return 0;
}
