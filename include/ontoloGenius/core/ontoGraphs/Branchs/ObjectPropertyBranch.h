#ifndef OBJETCPROPERTYBRANCH_H
#define OBJETCPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/PropertyBranch.h"

template <typename T>
class ObjectPropertyBranchData_t : public PropertyBranchData_t<T>
{
public:
  std::vector<T*> inverses_;
  std::vector<ClassBranch_t*> domains_;
  std::vector<ClassBranch_t*> ranges_;
  std::vector<std::vector<T*>> chains_;
};

class ObjectPropertyBranch_t :  public Branch_t<ObjectPropertyBranch_t>,
                                public ObjectPropertyBranchData_t<ObjectPropertyBranch_t>
{
public:
  ObjectPropertyBranch_t(std::string value) : Branch_t(value) {};
};

#endif
