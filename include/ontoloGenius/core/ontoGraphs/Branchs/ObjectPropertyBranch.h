#ifndef ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H
#define ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/PropertyBranch.h"

namespace ontologenius {

// Classes predefinition
class ObjectPropertyBranch_t;
class ClassBranch_t;

typedef Single_t<ObjectPropertyBranch_t*> ObjectPropertyElement_t;
typedef Single_t<ClassBranch_t*> ClassElement_t;

template <typename T>
class ObjectPropertyBranchData_t : public PropertyBranchData_t<T>
{
public:
  std::vector<T*> inverses_;

  std::vector<ClassElement_t> domains_;
  std::vector<ClassElement_t> ranges_;

  std::vector<std::vector<T*>> chains_;
  std::vector<std::vector<std::string>> str_chains_;
};

class ObjectPropertySteady_t :  public BranchSteady_t<ObjectPropertyBranch_t>,
                                public ObjectPropertyBranchData_t<ObjectPropertyBranch_t>
{
public:
  ObjectPropertySteady_t() {}
};

class ObjectPropertyBranch_t :  public Branch_t<ObjectPropertyBranch_t>,
                                public ObjectPropertyBranchData_t<ObjectPropertyBranch_t>
{
public:
  ObjectPropertySteady_t steady_;

  ObjectPropertyBranch_t(std::string value = "") : Branch_t(value) {};

  void setFullSteady();
  void setSteady_disjoint(ObjectPropertyBranch_t* disjoint);
  void setSteady_properties(Properties_t properties);
  void setSteady_inverse(ObjectPropertyBranch_t* inverse);
  void setSteady_domain(const ClassElement_t& domain);
  void setSteady_range(const ClassElement_t& range);
  void set_chain(std::vector<ObjectPropertyBranch_t*> chain);
  void setSteady_chain(std::vector<std::string> chain);
  void setSteady_child(const ObjectPropertyElement_t& child);
  void setSteady_mother(const ObjectPropertyElement_t& mother);
  void setSteady_dictionary(std::string lang, std::string word);
  void setSteady_muted_dictionary(std::string lang, std::string word);
  void setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
  void setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJETCPROPERTYBRANCH_H
