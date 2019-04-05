#ifndef ONTOLOGENIUS_VARIABLES_H
#define ONTOLOGENIUS_VARIABLES_H

#include <string>
#include <map>
#include <unordered_set>

#include <stdint.h>

#include "ontoloGenius/interpreter/codeDescription/Namespace.h"
#include "ontoloGenius/interpreter/codeDescription/Functions/FunctionContainer.h"

namespace ontologenius {

struct Variable_t
{
  std::string name;
  std::unordered_set<std::string> values;
};

class VariablesType : public Namespace, public FunctionContainer
{
public:
  VariablesType();
  ~VariablesType() {};

  std::string add(std::string name);

  std::string name(std::string id);
  std::unordered_set<std::string> get(std::string id);
  std::string toString(std::string id);

  bool set(std::string id, std::string value);
  bool set(std::string id, std::unordered_set<std::string> value);

  bool insert(std::string id, std::string value);
  bool insert(std::string id, std::unordered_set<std::string> value);

  bool remove(std::string id);
  bool remove(std::string id, std::string value);
  bool remove(std::string id, std::unordered_set<std::string> value);

  size_t size(std::string id);

private:
  uint16_t nb_variables_;

  std::map <std::string, Variable_t> var_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_VARIABLES_H
