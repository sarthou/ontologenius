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

  void removeComments();
  void getSubsections();
  void getIfBlock();
  size_t getNextIfBlock(int& nb_block, size_t pose);
  void getStrings();
  void getFromNamespace();

  Parser* subparser_;
  TreeObject& onto_;

  Code code_;
  Error error_;

  size_t begin_;
  size_t end_;
};

#endif
