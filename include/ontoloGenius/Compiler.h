#ifndef COMPILER_H
#define COMPILER_H

#include <string>
#include <map>
#include <set>

#include "ontoloGenius/codeDescription/Code.h"
#include "ontoloGenius/Error.h"

class Compiler
{
public:
  Compiler(Code* code);
  ~Compiler(){}

  size_t compile();
  Error& getError() {return error_; }

private:
  Code* code_;
  Error error_;

  static size_t section_cpt_;

  void compileIntructions(std::map<size_t, std::string> splited);
  std::map<size_t, std::string> splitBySemicolon();
  int getIfOffset(std::string ifelse_id);
  //int splitIfBlock(std::map<size_t, std::string>& splited, std::string ifelse_id);
};

#endif
