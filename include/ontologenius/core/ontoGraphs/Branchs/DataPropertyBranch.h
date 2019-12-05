#ifndef ONTOLOGENIUS_DATAPROPERTYBRANCH_H
#define ONTOLOGENIUS_DATAPROPERTYBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/PropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/Data.h"

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
  std::vector<data_t> ranges_;

  DataPropertyBranch_t(std::string value = "") : Branch_t(value) {};

  void setSteady_dictionary(std::string lang, std::string word);
  void setSteady_muted_dictionary(std::string lang, std::string word);
  void setSteady_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
  void setSteady_muted_dictionary(std::map<std::string, std::vector<std::string>> dictionary);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTYBRANCH_H
