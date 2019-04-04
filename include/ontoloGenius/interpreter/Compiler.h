#ifndef COMPILER_H
#define COMPILER_H

#include <string>
#include <map>
#include <unordered_set>
#include <fstream>

#include "ontoloGenius/interpreter/codeDescription/Code.h"
#include "ontoloGenius/interpreter/Error.h"
#include "ontoloGenius/interpreter/fileManagers/FileWriter.h"

namespace ontologenius {

class Compiler
{
public:
  Compiler(Code* code);
  ~Compiler();

  size_t compile();
  Error& getError() {return error_; }

private:
  Code* code_;
  Error error_;
  FileWriter writer_;

  static size_t section_cpt_;

  void compileIntructions(std::map<size_t, std::string> splited);
  type_t compileIntruction(std::string instruction, size_t pose);
  std::vector<type_t> compileParameters(std::string arg, size_t pose, FunctionDescriptor* descriptor);
  std::map<size_t, std::string> splitBySemicolon();
  int getIfOffset(std::string ifelse_id);

  type_t onVariableInstruction(std::string variable, std::string instruction, size_t pose);
  type_t onOntologyInstruction(std::string instruction, size_t pose);
  type_t onPropertyInstruction(std::string indiv, std::string instruction, size_t pose);

  void getParameters(std::string arg, size_t pose, std::vector<std::string>& args, std::vector<size_t>& args_pose);

  void noMatchigFunction(size_t pose, FunctionDescriptor* descriptor, std::vector<type_t> args_types);
  //int splitIfBlock(std::map<size_t, std::string>& splited, std::string ifelse_id);
};

} // namespace ontologenius

#endif
