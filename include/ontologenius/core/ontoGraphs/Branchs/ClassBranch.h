#ifndef ONTOLOGENIUS_CLASSBRANCH_H
#define ONTOLOGENIUS_CLASSBRANCH_H

#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/Branch.h"

#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"

namespace ontologenius {

// Classes predefinition
class ObjectPropertyBranch_t;
class DataPropertyBranch_t;
class IndividualBranch_t;
class ClassBranch_t;
class AnonymousClassBranch_t;

typedef Single_t<IndividualBranch_t*> IndividualElement_t;
typedef Single_t<ClassBranch_t*> ClassElement_t;
typedef Pair_t<ObjectPropertyBranch_t*, ClassBranch_t*> ClassObjectRelationElement_t;
typedef Pair_t<DataPropertyBranch_t*, LiteralNode*> ClassDataRelationElement_t;

class ClassBranch_t : public Branch_t<ClassBranch_t>
{
public:
  std::vector<IndividualElement_t> individual_childs_;
  std::vector<ClassElement_t> disjoints_;
  std::vector<ClassObjectRelationElement_t> object_relations_;
  std::vector<ClassDataRelationElement_t> data_relations_;
  std::vector<AnonymousClassBranch_t*> equiv_relations_;

  ClassBranch_t(const std::string& value = "") : Branch_t(value) {};

  void setSteady_dictionary(const std::string& lang, const std::string& word);
  void setSteady_muted_dictionary(const std::string& lang, const std::string& word);
  void setSteady_dictionary(const std::map<std::string, std::vector<std::string>>& dictionary);
  void setSteady_muted_dictionary(const std::map<std::string, std::vector<std::string>>& dictionary);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_CLASSBRANCH_H
