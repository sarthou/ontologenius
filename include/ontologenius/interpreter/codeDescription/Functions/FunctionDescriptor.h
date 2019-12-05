#ifndef ONTOLOGENIUS_FUNCTIONDESCRIPTOR_H
#define ONTOLOGENIUS_FUNCTIONDESCRIPTOR_H

#include <vector>
#include <string>

#include "ontologenius/interpreter/codeDescription/Types/Types.h"

namespace ontologenius {

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

  std::vector<std::vector<type_t>> getParameters() {return params_; }

private:
  std::string name_;
  std::string explicit_name_;
  std::vector<type_t> return_;
  std::vector<std::vector<type_t>> params_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_FUNCTIONDESCRIPTOR_H
