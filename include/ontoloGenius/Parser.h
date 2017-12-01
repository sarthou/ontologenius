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

struct LinesCounter_t
{
private:
  size_t start_;
  size_t stop_;
  size_t nb_lines_;

public:
  size_t getStart() {return start_;}
  size_t getStop() {return stop_;}
  size_t getNbLines() {return (nb_lines_ = (stop_ - start_ + 1));}
  void setStart(size_t start) {start_ = start;}
  void setStop(size_t stop) {stop_ = stop;}

  size_t current_line_;
};

struct IfBlock_t
{
  std::string IfBlock_condition;
  std::string IfBlock_if;
  std::string IfBlock_else;
  LinesCounter_t lines_count;
};

struct CommentBlock_t
{
  std::string comment;
  LinesCounter_t lines_count;
};

class Parser
{
public:
  Parser(std::string code, TreeObject& onto, size_t current_line = 1);
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
  size_t getLineNumber(size_t final_pose);
  size_t getBeginOfLine(size_t line_nb);
  void printCursor(size_t pose);

  ParserState parser_state_;
  Parser* subparser_;
  std::string code_;
  TreeObject& onto_;
  std::map<std::string, CommentBlock_t> comments_;
  std::map<std::string, std::string> subsections_;
  std::map<std::string, IfBlock_t> ifelse_;


  LinesCounter_t lines_counter_;
  size_t begin_;
  size_t end_;
};

#endif
