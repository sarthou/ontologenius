#ifndef ONTOLOGENIUS_DATAPROPERTYBRANCH_H
#define ONTOLOGENIUS_DATAPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

  // Classes predefinition
  class DataPropertyBranch;
  class ClassBranch;

  using DataPropertyElement = SingleElement<DataPropertyBranch*>;
  using ClassElement = SingleElement<ClassBranch*>;

  class DataPropertyBranch : public Branch<DataPropertyBranch>,
                             public PropertyBranch<DataPropertyBranch>
  {
  public:
    std::vector<ClassElement> domains_;
    std::vector<LiteralNode*> ranges_;

    DataPropertyBranch(const std::string& value = "", bool hidden = false) : Branch(value, hidden) {};
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTYBRANCH_H
