#ifndef ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H
#define ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

// Classes predefinition
class ObjectPropertyBranch_t;
class ClassBranch_t;

typedef Single_t<ObjectPropertyBranch_t*> ObjectPropertyElement_t;
typedef Single_t<ClassBranch_t*> ClassElement_t;

class ObjectPropertyBranch_t :  public Branch_t<ObjectPropertyBranch_t>,
                                public PropertyBranch_t<ObjectPropertyBranch_t>
{
public:
  std::vector<ObjectPropertyElement_t> inverses_;
  std::vector<ClassElement_t> domains_;
  std::vector<ClassElement_t> ranges_;
  std::vector<std::vector<ObjectPropertyBranch_t*>> chains_;
  std::vector<std::vector<std::string>> str_chains_;

  ObjectPropertyBranch_t(const std::string& value = "") : Branch_t(value) {};
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H
