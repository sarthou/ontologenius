#ifndef DATAPROPERTYBRANCH_T
#define DATAPROPERTYBRANCH_T

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/PropertyBranch.h"

template <typename T>
class DataPropertyBranchData_t : public PropertyBranchData_t<T>
{
public:
  std::vector<ClassBranch_t*> domains_;
  std::vector<std::string> ranges_;
};

class DataPropertyBranch_t :  public Branch_t<DataPropertyBranch_t>,
                              public DataPropertyBranchData_t<DataPropertyBranch_t>
{
public:
  DataPropertyBranch_t(std::string value) : Branch_t(value) {};
};


#endif
