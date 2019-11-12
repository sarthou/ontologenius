#ifndef ONTOLOGENIUS_DATAPROPERTYBRANCH_H
#define ONTOLOGENIUS_DATAPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/PropertyBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/Data.h"

namespace ontologenius {

// Classes predefinition
class DataPropertyBranch_t;
class ClassBranch_t;

typedef Single_t<DataPropertyBranch_t*> DataPropertyElement_t;
typedef Single_t<ClassBranch_t*> ClassElement_t;

template <typename T>
class DataPropertyBranchData_t : public PropertyBranchData_t<T>
{
public:
  std::vector<ClassElement_t> domains_;
  std::vector<data_t> ranges_;
};

class DataPropertySteady_t :  public BranchSteady_t<DataPropertyBranch_t>,
                              public DataPropertyBranchData_t<DataPropertyBranch_t>
{
public:
  DataPropertySteady_t() {}
};

class DataPropertyBranch_t :  public Branch_t<DataPropertyBranch_t>,
                              public DataPropertyBranchData_t<DataPropertyBranch_t>
{
public:
  DataPropertySteady_t steady_;

  DataPropertyBranch_t(std::string value = "") : Branch_t(value) {};

  void setFullSteady();
  void setSteady_disjoint(DataPropertyBranch_t* disjoint);
  void setSteady_properties(Properties_t properties);
  void setSteady_domain(const ClassElement_t& domain);
  void setSteady_range(std::string range);
  void setSteady_child(const DataPropertyElement_t& child);
  void setSteady_mother(const DataPropertyElement_t& mother);
  void setSteady_dictionary(std::string lang, std::string word);
  void setSteady_muted_dictionary(std::string lang, std::string word);
  void setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
  void setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTYBRANCH_H
