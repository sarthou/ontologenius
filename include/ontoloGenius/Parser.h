#ifndef PARSER_H
#define PARSER_H

#include <string>
#include <map>
#include <set>

#include "ontoloGenius/codeDescription/Code.h"

#include "ontoloGenius/TreeObject.h"
#include "ontoloGenius/Error.h"

class Parser
{
public:
  Parser(std::string code, TreeObject& onto, size_t current_line = 1);
  ~Parser();

private:
  void checkReserved();
  void checkReservedWord(std::string symbol);
  void checkBraquets();
  void checkStringAndComment();
  void getSubsections();
  void getIfBlock();
  size_t getNextIfBlock(int& nb_block, size_t pose);
  void getFromNamespace();
  void replaceOperator();
  std::map<size_t, std::string> splitBySemicolon();
  int splitIfBlock(std::map<size_t, std::string>& splited, std::string ifelse_id);
  void checkInstructionValidity(std::map<size_t, std::string>& splited);
  void checkInstructionValidity(size_t pose, std::string code, bool onFunction = false);
  void checkArgumentValidity(size_t pose, std::string code);

  Parser* subparser_;
  TreeObject& onto_;

  Code code_;
  Error error_;

  size_t begin_;
  size_t end_;
};

#endif
