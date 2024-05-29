#ifndef ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H
#define ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

  // Classes predefinition
  class ObjectPropertyBranch;
  class ClassBranch;

  typedef SingleElement<ObjectPropertyBranch*> ObjectPropertyElement;
  typedef SingleElement<ClassBranch*> ClassElement;

  class ObjectPropertyBranch : public Branch<ObjectPropertyBranch>,
                               public PropertyBranch<ObjectPropertyBranch>
  {
  public:
    std::vector<ObjectPropertyElement> inverses_;
    std::vector<ClassElement> domains_;
    std::vector<ClassElement> ranges_;
    std::vector<std::vector<ObjectPropertyBranch*>> chains_;
    std::vector<std::vector<std::string>> str_chains_;

    ObjectPropertyBranch(const std::string& value = "") : Branch(value){};
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H
