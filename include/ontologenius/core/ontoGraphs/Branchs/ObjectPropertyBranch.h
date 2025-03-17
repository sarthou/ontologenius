#ifndef ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H
#define ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

  // Classes predefinition
  class ObjectPropertyBranch;
  class ClassBranch;

  using ObjectPropertyElement = SingleElement<ObjectPropertyBranch*>;
  using ClassElement = SingleElement<ClassBranch*>;

  class ObjectPropertyBranch : public Branch<ObjectPropertyBranch>,
                               public PropertyBranch<ObjectPropertyBranch>
  {
  public:
    std::vector<ObjectPropertyElement> inverses_;
    std::vector<ClassElement> domains_;
    std::vector<ClassElement> ranges_;
    std::vector<std::vector<ObjectPropertyBranch*>> chains_;
    std::vector<std::vector<std::string>> str_chains_;

    ObjectPropertyBranch(const std::string& value = "", bool hidden = false) : Branch(value, hidden) {};
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H
