#ifndef INDIVIDUALBRANCH_H
#define INDIVIDUALBRANCH_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/ValuedNode.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

template <typename T>
class IndividualBranchData_t
{
public:
  std::vector<ClassBranch_t*> is_a_;

  std::vector<ObjectPropertyBranch_t*> object_properties_name_;
  std::vector<T*> object_properties_on_;

  std::vector<DataPropertyBranch_t*> data_properties_name_;
  std::vector<data_t> data_properties_data_;

  std::vector<T*> same_as_;
  std::vector<T*> distinct_;
};

class IndividualBranch_t : public ValuedNode, public IndividualBranchData_t<IndividualBranch_t>
{
public:
  bool mark;

  IndividualBranch_t(std::string value) : ValuedNode(value) {mark = false; }
};

#endif
