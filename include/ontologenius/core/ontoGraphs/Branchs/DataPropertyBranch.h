#ifndef ONTOLOGENIUS_DATAPROPERTYBRANCH_H
#define ONTOLOGENIUS_DATAPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"

namespace ontologenius {

// Classes predefinition
class DataPropertyBranch_t;
class ClassBranch_t;

typedef Single_t<DataPropertyBranch_t*> DataPropertyElement_t;
typedef Single_t<ClassBranch_t*> ClassElement_t;

class DataPropertyBranch_t :  public Branch_t<DataPropertyBranch_t>,
                              public PropertyBranch_t<DataPropertyBranch_t>
{
public:
  std::vector<ClassElement_t> domains_;
  std::vector<LiteralNode*> ranges_;

  DataPropertyBranch_t(const std::string& value = "") : Branch_t(value) {};
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTYBRANCH_H
