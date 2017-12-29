#ifndef VARIABLES_H
#define VARIABLES_H

#include <string>
#include <map>
#include <set>

#include <stdint.h>

#include "ontoloGenius/codeDescription/Namespace.h"
#include "ontoloGenius/codeDescription/FunctionContainer.h"

struct Variable_t
{
  std::string name;
  std::set<std::string> values;
};

class Variables : public Namespace, public FunctionContainer
{
public:
  Variables();
  ~Variables() {};

  std::string add(std::string name);

  std::string name(std::string id);
  std::set<std::string> get(std::string id);
  std::string toString(std::string id);

  bool set(std::string id, std::string value);
  bool set(std::string id, std::set<std::string> value);

  bool insert(std::string id, std::string value);
  bool insert(std::string id, std::set<std::string> value);

  bool remove(std::string id);
  bool remove(std::string id, std::string value);
  bool remove(std::string id, std::set<std::string> value);

  size_t size(std::string id);

private:
  uint16_t nb_variables_;

  std::map <std::string, Variable_t> var_;
};

#endif
