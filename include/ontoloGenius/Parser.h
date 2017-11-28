#ifndef PARSER_H
#define PARSER_H

#include <string>
#include <map>

#include "ontoloGenius/TreeObject.h"

enum class ParserState
{
  wait,
  assignment,
  loop,
  instruction,
  condition_if,
  condition_if_true,
  condition_if_false
};

struct IfBlock_t
{
  std::string IfBlock_condition;
  std::string IfBlock_if;
  std::string IfBlock_else;
};

class Parser
{
public:
  Parser(std::string code, TreeObject& onto);
  ~Parser();

  ParserState getState() const;
  bool move();

private:
  bool if_condition();
  void loop();
  void instruction();

  void removeComments();

  void getSubsections();
  void getIfBlock();
  size_t getNextIfBlock(int& nb_block, size_t pose);

  size_t getInBraquet(size_t begin, std::string& in_braquet);
  bool findBefore(size_t begin, char symbol);
  size_t findAfter(size_t begin, std::string symbol);

  ParserState parser_state_;
  Parser* subparser_;
  std::string code_;
  TreeObject& onto_;
  std::map<std::string, std::string> subsections_;
  std::map<std::string, IfBlock_t> ifelse_;

  size_t begin_;
  size_t end_;
};

#endif
