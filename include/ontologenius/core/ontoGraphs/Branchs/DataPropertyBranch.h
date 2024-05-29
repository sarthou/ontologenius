#ifndef ONTOLOGENIUS_DATAPROPERTYBRANCH_H
#define ONTOLOGENIUS_DATAPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

  // Classes predefinition
  class DataPropertyBranch;
  class ClassBranch;

  typedef SingleElement<DataPropertyBranch*> DataPropertyElement;
  typedef SingleElement<ClassBranch*> ClassElement;

  class DataPropertyBranch : public Branch<DataPropertyBranch>,
                             public PropertyBranch<DataPropertyBranch>
  {
  public:
    std::vector<ClassElement> domains_;
    std::vector<LiteralNode*> ranges_;

    DataPropertyBranch(const std::string& value = "") : Branch(value){};
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTYBRANCH_H
