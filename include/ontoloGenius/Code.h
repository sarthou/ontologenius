#ifndef CODE_H
#define CODE_H

#include <string>
#include <map>
#include <vector>

#include <iostream>

#include <stdint.h>

#include "ontoloGenius/Variables.h"

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

struct SubsectionBlock_t
{
  std::string subsection;
  LinesCounter_t lines_count;
};

struct StringBlock_t
{
  std::string strings;
  LinesCounter_t lines_count;
};

struct FunctionBlock_t
{
  std::string name;
  std::vector<std::string> params;
};

class Code
{
public:
  Code(std::string code) {text = code; }
  ~Code() {}

  std::string text;
  LinesCounter_t lines_counter_;

  std::map<std::string, CommentBlock_t> comments_;
  std::map<std::string, SubsectionBlock_t> subsections_;
  std::map<std::string, IfBlock_t> ifelse_;
  std::map<std::string, StringBlock_t> strings_;
  Variables variables_;
  std::map<std::string, FunctionBlock_t> functions_;

  size_t getInBraquet(size_t begin, std::string& in_braquet, std::string& text);
  bool findBefore(size_t begin, char symbol);
  bool findJustBefore(size_t begin, std::string symbol);
  std::string getWordBefore(size_t begin);
  std::string getWordAfter(size_t begin);
  size_t findAfter(size_t begin, std::string symbol);
  bool findHere(size_t begin, std::string symbol);

  size_t getLineNumber(size_t final_pose);
  size_t getNbOfSublines(size_t& current_pose, size_t stop = -1);

  void print() {std::cout << text << std::endl; }
  void remove(char character);

private:
};

#endif
