#ifndef FUNCTIONDESCRIPTOR_H
#define FUNCTIONDESCRIPTOR_H

#include <vector>
#include <string>

#include "ontoloGenius/codeDescription/Types/Types.h"

class FunctionDescriptor
{
public:
  FunctionDescriptor(std::string name, type_t return_type, std::vector<type_t> params_type);
  ~FunctionDescriptor() {}

  bool overload(type_t return_type, std::vector<type_t> params_type);

  void addExplicitName(std::string name) { explicit_name_ = name; }
  std::string getExplicitName();
  std::string getName();
  type_t getReturnType(std::vector<type_t> params);

  bool testParams(std::vector<type_t> params);
  size_t testNbParams(size_t nb);
  std::string getDeclaration(size_t nb);

  std::string to_string(type_t type);

private:
  std::string name_;
  std::string explicit_name_;
  std::vector<type_t> return_;
  std::vector<std::vector<type_t>> params_;
};

#endif
